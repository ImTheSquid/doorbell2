use core::pin::pin;

use std::sync::Arc;

use chrono::{Datelike, TimeZone, Utc};
use chrono_tz::America::Los_Angeles;
use doorbell2::door_lock::{self, ClusterAsyncHandler};
use doorbell2::{run_status_led, NetStatus, SolenoidHandler};
use esp_idf_matter::init_async_io;
use esp_idf_matter::matter::crypto::{default_crypto, Crypto};
use esp_idf_matter::matter::dm::clusters::desc::{self, ClusterHandler as _, DescHandler};
use esp_idf_matter::matter::dm::devices::test::{DAC_PRIVKEY, TEST_DEV_ATT};
use esp_idf_matter::matter::dm::{
    Async, Dataver, DeviceType, EmptyHandler, Endpoint, EpClMatcher, Node,
};
use esp_idf_matter::matter::utils::init::InitMaybeUninit;
use esp_idf_matter::matter::utils::sync::blocking::raw::StdRawMutex;
use esp_idf_matter::matter::{clusters, devices};
use esp_idf_matter::stack::{nal, UserTask};
use esp_idf_matter::wireless::{EspMatterWifi, EspWifiMatterStack};

use esp_idf_svc::bt::reduce_bt_memory;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::gpio::PinDriver;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::task::block_on;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::io::vfs::MountedEventfs;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::timer::EspTaskTimerService;

use log::{error, info, warn};

use rs_matter::crypto::{CryptoSensitive, CryptoSensitiveRef};
use rs_matter::dm::clusters::basic_info::BasicInfoConfig;
use rs_matter::BasicCommData;

use std::sync::atomic::{AtomicBool, AtomicU8};

const STACK_SIZE: usize = 36 * 1024;
const BLUETOOTH_STACK_SIZE: usize = 20 * 1024;

/// WebSocket server URI to connect to for external unlock commands
// const WS_SERVER_URI: &str = "ws://192.168.68.1:8080";

pub const DEVICE_CONFIG: BasicInfoConfig = BasicInfoConfig {
    vid: 0xdead,
    pid: 0xbeef,
    hw_ver: 1,
    hw_ver_str: "1",
    sw_ver: 1,
    sw_ver_str: "1",
    serial_no: "10x+5i",
    product_name: "doorbell2",
    vendor_name: "purdue hackers",
    device_name: "doorbell2",
    ..BasicInfoConfig::new()
};

/// Mostly stolen from https://github.com/sysgrok/esp-idf-matter/blob/master/examples/light_wifi.rs
fn main() -> Result<(), anyhow::Error> {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Booting solenoid lock controller...");

    ThreadSpawnConfiguration::set(&ThreadSpawnConfiguration {
        name: Some(c"matter"),
        ..Default::default()
    })?;

    // Run in a higher-prio thread to avoid issues with `async-io` getting
    // confused by the low priority of the ESP IDF main task
    // Also allocate a very large stack (for now) as `rs-matter` futures do occupy quite some space
    let thread = std::thread::Builder::new()
        .stack_size(STACK_SIZE)
        .spawn(run)
        .unwrap();

    thread.join().unwrap()
}

#[inline(never)]
#[cold]
fn run() -> Result<(), anyhow::Error> {
    let result = block_on(matter());

    if let Err(e) = &result {
        error!("Matter aborted execution with error: {e:?}");
    }
    {
        info!("Matter finished execution successfully");
    }

    result
}

async fn matter() -> Result<(), anyhow::Error> {
    // Initialize the Matter stack (can be done only once),
    // as we'll run it in this thread
    // Heap-allocate the ~64 KB Matter stack instead of putting it in static
    // `.bss`. Static DRAM (`dram0_0_seg`) is only ~124 KB on this ESP32 after
    // the BT controller's reservation, and this buffer alone is over half of it;
    // the runtime heap is far larger. `new_uninit` allocates straight on the
    // heap with no 64 KB stack temporary, and `Box::leak` gives it the `'static`
    // lifetime the WifiBle stack requires.
    let stack =
        Box::leak(Box::<EspWifiMatterStack<'static, BLUETOOTH_STACK_SIZE, ()>>::new_uninit())
            .init_with(EspWifiMatterStack::init_default(
                &DEVICE_CONFIG,
                BasicCommData {
                    password: CryptoSensitive::new_from_ref(CryptoSensitiveRef::new(
                        &31415926_u32.to_le_bytes(),
                    )),
                    discriminator: 1229,
                },
                &TEST_DEV_ATT,
            ));

    // Take some generic ESP-IDF stuff we'll need later
    let sysloop = EspSystemEventLoop::take()?;
    let timers = EspTaskTimerService::new()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let mut peripherals = Peripherals::take()?;

    let mounted_event_fs = Arc::new(MountedEventfs::mount(3)?);
    init_async_io(mounted_event_fs.clone())?;

    reduce_bt_memory(unsafe { peripherals.modem.reborrow() })
        .expect("bt memory reduction successful");

    // Create the default crypto provider using the STD CSPRNG provided by the `rand` crate
    let crypto = default_crypto::<StdRawMutex, _>(rand::thread_rng(), DAC_PRIVKEY);

    let mut good_rand = crypto.rand()?;

    let unlock_request = Arc::new(AtomicBool::new(false));

    // Connectivity phase surfaced on the GPIO23 status LED (updated by
    // `WifiStabilityTask`). Lock state has its own dedicated LEDs on the board.
    let net_status = Arc::new(AtomicU8::new(NetStatus::Booting as u8));

    let solenoid = SolenoidHandler::new(
        Dataver::new_rand(&mut good_rand),
        SOLENOID_ENDPOINT_ID,
        PinDriver::input_output(peripherals.pins.gpio16, esp_idf_svc::hal::gpio::Pull::Down)
            .expect("GPIO 16 init"),
        PinDriver::input_output(peripherals.pins.gpio17, esp_idf_svc::hal::gpio::Pull::Down)
            .expect("GPIO 17 init"),
        unlock_request.clone(),
    );

    // Chain our endpoint clusters with the
    // (root) Endpoint 0 system clusters in the final handler
    let handler = EmptyHandler
        // Our on-off cluster, on Endpoint 1
        .chain(
            EpClMatcher::new(
                Some(SOLENOID_ENDPOINT_ID),
                Some(SolenoidHandler::CLUSTER.id),
            ),
            door_lock::HandlerAsyncAdaptor(&solenoid),
        )
        // Each Endpoint needs a Descriptor cluster too
        // Just use the one that `rs-matter` provides out of the box
        .chain(
            EpClMatcher::new(Some(SOLENOID_ENDPOINT_ID), Some(DescHandler::CLUSTER.id)),
            Async(desc::DescHandler::new(Dataver::new_rand(&mut good_rand)).adapt()),
        );

    // Create the persister & load any previously saved state
    // `EspKvBlobStore` saves to a user-supplied ESP-IDF NVS partition
    // However, for this demo and for simplicity, we use a dummy persister that does nothing
    let persist = stack
        .create_persist_with_comm_window(
            &crypto,
            esp_idf_matter::persist::EspKvBlobStore::new_default(nvs.clone())
                .expect("valid NVS partition"),
        )
        .await?;

    // Run the Matter stack with our handler
    let matter = stack.run_coex(
        // The Matter stack needs the Wifi/BLE modem peripheral
        EspMatterWifi::new_with_builtin_mdns(peripherals.modem, sysloop, timers, nvs, stack),
        // The Matter stack needs a persister to store its state
        &persist,
        // The crypto provider
        &crypto,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // Tune WiFi for stability with BLE coex, reporting phase on the LED
        WifiStabilityTask {
            stack,
            _unlock_request: unlock_request,
            net_status: net_status.clone(),
        },
    );

    let led = PinDriver::output(peripherals.pins.gpio23).expect("GPIO 23 init");
    let led_task = run_status_led(led, net_status.clone());

    // Run Matter and the status-LED driver concurrently. `led_task` loops
    // forever; only the Matter future can finish (on error), so propagate that.
    // `pin!` keeps the (large) Matter future off the stack as it did before.
    match embassy_futures::select::select(pin!(matter), pin!(led_task)).await {
        embassy_futures::select::Either::First(res) => res?,
        embassy_futures::select::Either::Second(()) => {}
    }

    Ok(())
}

/// Tunes WiFi for stability when coexisting with BLE on ESP32's shared radio.
/// Disables WiFi PS immediately, then polls for commissioning completion
/// before shutting down BLE to free the shared radio.
struct WifiStabilityTask {
    stack: &'static EspWifiMatterStack<'static, BLUETOOTH_STACK_SIZE, ()>,
    _unlock_request: Arc<AtomicBool>,
    /// Shared phase indicator driving the status LED.
    net_status: Arc<AtomicU8>,
}

impl UserTask for WifiStabilityTask {
    async fn run<S, N>(&mut self, _net_stack: S, netif: N) -> Result<(), rs_matter::error::Error>
    where
        S: nal::NetStack,
        N: rs_matter::dm::clusters::gen_diag::NetifDiag + rs_matter::dm::networks::NetChangeNotif,
    {
        // Matter needs a responsive link. WiFi's default is modem-sleep power save
        // (`pm start, type: 1`), which drops/delays UDP and makes commissioning
        // joins flaky — disable it as early as possible. (Coex preference is set
        // later, once the commissioning network is up.)
        unsafe {
            info!("Disabling WiFi power save");
            esp_idf_svc::sys::esp_wifi_set_ps(esp_idf_svc::sys::wifi_ps_type_t_WIFI_PS_NONE);
        }

        let commissioned = self.stack.matter().is_commissioned();
        info!("Commissioned state at UserTask start: {}", commissioned);

        // Definitive readout of *why* we're (un)commissioned: `is_commissioned()`
        // is just "fabric count > 0", so dump the actual fabrics. A previously
        // paired device showing 0 fabrics here means it lost/never persisted its
        // fabric (vs. just failing to connect).
        {
            let matter = self.stack.matter();
            let fabrics = matter.fabric_mgr.borrow();
            info!("Persisted fabric count: {}", fabrics.iter().count());
            for fabric in fabrics.iter() {
                info!(
                    "  fabric idx={} fabric_id={:#x} node_id={:#x} label={:?}",
                    fabric.fab_idx(),
                    fabric.fabric_id(),
                    fabric.node_id(),
                    fabric.label(),
                );
            }
        }

        // Already paired -> we're just reconnecting WiFi; otherwise advertise.
        if commissioned {
            NetStatus::WifiConnecting.store(&self.net_status);
        } else {
            NetStatus::Commissioning.store(&self.net_status);
        }

        if !commissioned {
            info!("Waiting for commissioning to complete...");

            // Wait until commissioned
            loop {
                netif.wait_changed().await;
                if self.stack.matter().is_commissioned() {
                    break;
                }
            }

            // Commissioned — wait for one more netif change so the WiFi credentials
            // exchange over BLE completes and WiFi actually connects
            info!("Commissioned, waiting for WiFi to connect...");
            NetStatus::WifiConnecting.store(&self.net_status);
            netif.wait_changed().await;
        }

        // Fully tear BLE down (not just deprioritize coex). The commissioner has
        // the WiFi creds by now and finishes over CASE/WiFi, so BLE is no longer
        // needed — and keeping the controller + bluedroid resident holds tens of
        // KB of RAM. That RAM is needed for the WiFi/LWIP TX path: the post-CASE
        // subscription ReportData burst to Apple's two fabrics is large, and with
        // BLE still resident the sends fail with ENOMEM ("Not enough space"),
        // stalling commissioning. Tearing BLE down frees that memory.
        info!("Shutting down BLE to stabilize WiFi and free memory");
        unsafe {
            esp_idf_svc::sys::esp_bluedroid_disable();
            esp_idf_svc::sys::esp_bluedroid_deinit();
            esp_idf_svc::sys::esp_bt_controller_disable();
            esp_idf_svc::sys::esp_bt_controller_deinit();
            esp_idf_svc::sys::esp_coex_preference_set(
                esp_idf_svc::sys::esp_coex_prefer_t_ESP_COEX_PREFER_WIFI,
            );
        }
        info!("BLE shutdown complete, WiFi has exclusive radio access");
        NetStatus::Online.store(&self.net_status);

        // // Start WebSocket client to listen for external unlock commands
        // info!("Connecting to WebSocket server: {}", WS_SERVER_URI);
        // let unlock = self.unlock_request.clone();
        // let _ws_client = esp_idf_svc::ws::client::EspWebSocketClient::new(
        //     WS_SERVER_URI,
        //     &esp_idf_svc::ws::client::EspWebSocketClientConfig::default(),
        //     core::time::Duration::from_secs(10),
        //     move |event| {
        //         if let Ok(event) = event {
        //             match event.event_type {
        //                 esp_idf_svc::ws::client::WebSocketEventType::Connected => {
        //                     info!("WebSocket connected");
        //                 }
        //                 esp_idf_svc::ws::client::WebSocketEventType::Disconnected => {
        //                     warn!("WebSocket disconnected");
        //                 }
        //                 esp_idf_svc::ws::client::WebSocketEventType::Text(text) => {
        //                     info!("WebSocket received: {}", text);
        //                     if text == "open" {
        //                         info!("WebSocket: unlock command received");
        //                         unlock.store(true, Ordering::Relaxed);
        //                     }
        //                 }
        //                 _ => {}
        //             }
        //         }
        //     },
        // );

        // match _ws_client {
        //     Ok(_client) => {
        //         info!("WebSocket client started");
        //         // Keep the client alive by holding it in scope while we await forever
        //         core::future::pending().await
        //     }
        //     Err(e) => {
        //         error!("Failed to start WebSocket client: {:?}", e);
        //         core::future::pending().await
        //     }
        // }

        // The device becomes unresponsive to HomeKit every 40ish hours.
        // Sync the clock over SNTP, then reboot at the nearest upcoming
        // Pacific midnight (a quiet hour) to clear the hang before it happens.
        //
        // rs-matter times its fail-safe off the wall clock (`sys_epoch`), so a
        // large forward `settimeofday` step while a fail-safe is armed instantly
        // expires it and rolls back commissioning. A fail-safe is armed only
        // during a *fresh* commission. So the one unsafe case is: we booted
        // uncommissioned (fresh pairing) AND the clock is still bogus (≈1970 — a
        // cold power-on with no RTC time to carry across the reboot). In that
        // case a sync would hard-step mid-pairing and kill it, so we defer to the
        // next boot, which comes up already-commissioned (no fail-safe armed).
        // This is a plain boolean guard, not a delay. Every other boot syncs.
        let clock_was_bogus = Utc::now().year() < 2026;
        let synced = if !commissioned && clock_was_bogus {
            warn!("Fresh pairing on an unsynced clock; deferring SNTP to the next boot so the clock step can't roll back commissioning");
            false
        } else {
            // Smooth mode slews sub-35-min deltas via `adjtime` instead of
            // stepping, so an already-timed device's correction never trips the
            // fail-safe even if one happens to be armed (e.g. a re-commission).
            // A bogus-clock device here is already commissioned (no fail-safe), so
            // the unavoidable hard step is harmless.
            info!("Starting SNTP time sync (smooth mode)");
            let conf = esp_idf_svc::sntp::SntpConf {
                sync_mode: esp_idf_svc::sntp::SyncMode::Smooth,
                ..Default::default()
            };
            match esp_idf_svc::sntp::EspSntp::new(&conf) {
                Ok(sntp) => {
                    // Block (bounded) until SNTP has actually set the system clock.
                    let mut waited_secs = 0u32;
                    while sntp.get_sync_status() != esp_idf_svc::sntp::SyncStatus::Completed {
                        if waited_secs >= 120 {
                            warn!("SNTP did not sync within 120s; using fallback reboot timer");
                            break;
                        }
                        embassy_time::Timer::after_secs(1).await;
                        waited_secs += 1;
                    }
                    // Keep `sntp` alive for the rest of the task; dropping it stops sync.
                    core::mem::forget(sntp);
                    Utc::now().year() >= 2025
                }
                Err(e) => {
                    error!("Failed to start SNTP: {e:?}");
                    false
                }
            }
        };

        // If the clock just hard-stepped (only reachable on an already-commissioned
        // boot — fresh pairings are deferred above), drop now-stale Matter sessions
        // so peers re-establish CASE with fresh timestamps. No fail-safe is armed
        // in that case, so this only churns sessions, never fabrics. On this
        // single-threaded executor `reset_transport` can't race a transport borrow;
        // the only expected failure is `InvalidState` (packet in flight), retried.
        if synced && clock_was_bogus {
            info!(
                "Clock stepped forward; resetting Matter transport to clear stale session timers"
            );
            let mut attempts = 0u32;
            loop {
                match self.stack.matter().reset_transport() {
                    Ok(()) => {
                        info!("Matter transport reset after clock step");
                        break;
                    }
                    Err(e) => {
                        attempts += 1;
                        if attempts >= 10 {
                            warn!("Could not reset transport after {attempts} attempts: {e:?}");
                            break;
                        }
                        embassy_time::Timer::after_millis(200).await;
                    }
                }
            }
        }

        let reboot_point = if synced {
            // Nearest upcoming midnight in America/Los_Angeles, DST-aware.
            // Midnight never falls in a DST transition (those happen at 02:00),
            // so the local time is always unambiguous.
            let now_pacific = Utc::now().with_timezone(&Los_Angeles);
            let next_date = now_pacific
                .date_naive()
                .succ_opt()
                .expect("date has a successor");
            let next_midnight = next_date.and_hms_opt(0, 0, 0).expect("00:00:00 is valid");
            let point = Los_Angeles
                .from_local_datetime(&next_midnight)
                .single()
                .expect("midnight is unambiguous")
                .with_timezone(&Utc);
            info!(
                "Scheduling reboot at next Pacific midnight: {} Pacific ({} UTC)",
                next_midnight, point
            );
            point
        } else {
            // Clock never synced: fall back to a relative 24h timer. This still
            // works with an unsynced (1970-based) clock since both sides of the
            // comparison use the same monotonic-from-boot system time.
            warn!("Clock unsynced; falling back to reboot ~24h from now");
            Utc::now() + chrono::Duration::hours(24)
        };

        while Utc::now() < reboot_point {
            embassy_time::Timer::after_secs(60).await;
        }
        info!("Scheduled time reached, rebooting...");
        unsafe {
            esp_idf_svc::sys::esp_restart();
        }
    }
}

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const SOLENOID_ENDPOINT_ID: u16 = 1;

/// The Matter Light device Node
const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EspWifiMatterStack::<0, ()>::root_endpoint(),
        Endpoint {
            id: SOLENOID_ENDPOINT_ID,
            device_types: devices!(DeviceType {
                dtype: 0xA,
                drev: 3
            }),
            clusters: clusters!(SolenoidHandler::CLUSTER, DescHandler::CLUSTER),
        },
    ],
};
