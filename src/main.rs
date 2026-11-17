mod persist;

use core::pin::pin;
use embassy_time::Timer;

use persist::MatterBlobStore;
use std::sync::{
    atomic::{AtomicBool, AtomicU8},
    Arc,
};

use chrono::{Datelike, TimeZone, Utc};
use chrono_tz::America::Los_Angeles;
use doorbell2::door_lock::{self, ClusterAsyncHandler};
use doorbell2::{run_status_led, NetStatus, SolenoidHandler};
use esp_idf_matter::init_async_io;
use esp_idf_matter::matter::crypto::{default_crypto, Crypto};
use esp_idf_matter::matter::dm::clusters::desc::{self, ClusterHandler as _, DescHandler};
use esp_idf_matter::matter::dm::devices::test::{DAC_PRIVKEY, TEST_DEV_ATT};
use esp_idf_matter::matter::dm::endpoints::ROOT_ENDPOINT_ID;
use esp_idf_matter::matter::dm::{Async, Dataver, DeviceType, EmptyHandler, Endpoint, Node};
use esp_idf_matter::matter::stack::net::NetStack;
use esp_idf_matter::matter::stack::wireless::BTP_BUMP_SIZE;
use esp_idf_matter::matter::stack::{MatterStack, UserTask};
use esp_idf_matter::matter::utils::futures::{select, select4, Coalesce, Either};
use esp_idf_matter::matter::utils::init::InitMaybeUninit;
use esp_idf_matter::matter::{clusters, devices, root_endpoint};
use esp_idf_matter::wireless::{EspMatterWifi, EspWifiNetwork};

use esp_idf_svc::bt::reduce_bt_memory;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::gpio::{PinDriver, Pull};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::task::block_on;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::io::vfs::MountedEventfs;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::timer::EspTaskTimerService;
use esp_idf_svc::{
    sntp::{EspSntp, SntpConf, SyncMode, SyncStatus},
    sys,
};
use rs_matter::{
    dm::{clusters::gen_diag::NetifDiag, networks::NetChangeNotif},
    error::Error as MatterError,
    session::SessionMode,
};

use log::{error, info, warn};

use rs_matter::dm::clusters::basic_info::BasicInfoConfig;
use rs_matter::pairing::{BasicCommData, Spake2pVerifierPassword, Spake2pVerifierPasswordRef};

const STACK_SIZE: usize = 36 * 1024;
const BUMP_SIZE: usize = 20 * 1024 + BTP_BUMP_SIZE;
const DIAG_INTERVAL_SECS: u64 = 300;
type Stack = MatterStack<'static, BUMP_SIZE, EspWifiNetwork>;

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

fn main() -> Result<(), anyhow::Error> {
    sys::link_patches();

    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Booting solenoid lock controller...");

    ThreadSpawnConfiguration::set(&ThreadSpawnConfiguration {
        name: Some(c"matter"),
        ..Default::default()
    })?;

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
    info!("Matter finished execution successfully");

    result
}

async fn matter() -> Result<(), anyhow::Error> {
    // Heap allocation avoids exhausting static DRAM; the stack needs a static lifetime.
    let stack = Box::leak(Box::<Stack>::new_uninit()).init_with(Stack::init(
        &DEVICE_CONFIG,
        BasicCommData {
            password: Spake2pVerifierPassword::new_from_ref(Spake2pVerifierPasswordRef::new(
                &31415926_u32.to_le_bytes(),
            )),
            discriminator: 1229,
        },
        &TEST_DEV_ATT,
    ));

    let sysloop = EspSystemEventLoop::take()?;
    let timers = EspTaskTimerService::new()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let mut peripherals = Peripherals::take()?;

    let mounted_event_fs = Arc::new(MountedEventfs::mount(3)?);
    init_async_io(mounted_event_fs.clone())?;

    reduce_bt_memory(unsafe { peripherals.modem.reborrow() })
        .expect("bt memory reduction successful");

    let crypto = default_crypto(rand::rng(), DAC_PRIVKEY);

    let mut weak_rand = crypto.weak_rand()?;

    let unlock_request = Arc::new(AtomicBool::new(false));

    let net_status = Arc::new(AtomicU8::new(NetStatus::Booting as u8));

    let solenoid = SolenoidHandler::new(
        Dataver::new_rand(&mut weak_rand),
        SOLENOID_ENDPOINT_ID,
        PinDriver::input_output(peripherals.pins.gpio16, Pull::Down).expect("GPIO 16 init"),
        PinDriver::input_output(peripherals.pins.gpio17, Pull::Down).expect("GPIO 17 init"),
        unlock_request.clone(),
    );

    let handler = EmptyHandler
        .chain(
            |e, _| e == ROOT_ENDPOINT_ID,
            Async(Stack::root_handler(stack.matter(), &(), &mut weak_rand)),
        )
        .chain(
            |e, c| e == SOLENOID_ENDPOINT_ID && c == SolenoidHandler::CLUSTER.id,
            door_lock::HandlerAsyncAdaptor(&solenoid),
        )
        .chain(
            |e, c| e == SOLENOID_ENDPOINT_ID && c == DescHandler::CLUSTER.id,
            Async(desc::DescHandler::new(Dataver::new_rand(&mut weak_rand)).adapt()),
        );

    let mut store = MatterBlobStore::new(nvs.clone())?;
    stack.startup(&crypto, &mut store).await?;
    // Capture restored state before commissioning can change the fabric table.
    let had_fabric_at_start = stack.matter().has_fabrics();

    let matter = stack.run_wifi(
        EspMatterWifi::new_with_builtin_mdns(peripherals.modem, sysloop, timers, nvs, stack),
        &crypto,
        (NODE, &handler),
        stack.kv(&mut store),
        WifiStabilityTask {
            stack,
            had_fabric_at_start,
            net_status: net_status.clone(),
        },
    );

    let led = PinDriver::output(peripherals.pins.gpio23).expect("GPIO 23 init");
    let led_task = run_status_led(led, net_status.clone());

    match select(pin!(matter), pin!(led_task)).await {
        Either::First(res) => res?,
        Either::Second(()) => {}
    }

    Ok(())
}

struct WifiStabilityTask {
    stack: &'static Stack,
    had_fabric_at_start: bool,
    net_status: Arc<AtomicU8>,
}

impl UserTask for WifiStabilityTask {
    async fn run<S, N>(&mut self, _net_stack: S, netif: N) -> Result<(), MatterError>
    where
        S: NetStack,
        N: NetifDiag + NetChangeNotif,
    {
        info!("Disabling WiFi power save");
        let result = unsafe { sys::esp_wifi_set_ps(sys::wifi_ps_type_t_WIFI_PS_NONE) };
        if let Some(err) = sys::EspError::from(result) {
            warn!("Could not disable WiFi power save: {err}");
        }

        info!("Fabric restored at startup: {}", self.had_fabric_at_start);

        self.stack.matter().with_state(|state| {
            info!("Current fabric count: {}", state.fabrics.iter().count());
            for fabric in state.fabrics.iter() {
                info!(
                    "  fabric idx={} fabric_id={:#x} node_id={:#x} label={:?}",
                    fabric.fab_idx(),
                    fabric.fabric_id(),
                    fabric.node_id(),
                    fabric.label(),
                );
            }
        });

        select4(
            self.monitor_connectivity(&netif),
            self.manage_ble(),
            self.run_maintenance(),
            self.log_diagnostics(),
        )
        .coalesce()
        .await
    }
}

impl WifiStabilityTask {
    /// A CASE session means a controller reached us over WiFi. The stack runs
    /// the BLE peripheral while a self-opened commissioning window is open, so
    /// wait for that to close too: the stack has then stopped advertising.
    async fn manage_ble(&self) -> Result<(), MatterError> {
        info!("Keeping BLE up until a controller connects over CASE");
        while !self.has_case_session()
            || self
                .stack
                .matter()
                .comm_window_state()
                .is_open_on_all_transports()
        {
            Timer::after_millis(200).await;
        }
        shutdown_ble();

        // BLE cannot come back without a reboot, which re-pairing needs.
        loop {
            Timer::after_secs(5).await;
            if !self.stack.matter().has_fabrics() {
                warn!("No fabrics left; rebooting to re-enable BLE commissioning");
                // Let the RemoveFabric response go out first.
                Timer::after_secs(5).await;
                unsafe { sys::esp_restart() };
            }
        }
    }

    fn has_case_session(&self) -> bool {
        self.stack.matter().with_state(|state| {
            state.sessions.iter().any(|s| {
                !s.is_reserved() && matches!(s.get_session_mode(), SessionMode::Case { .. })
            })
        })
    }

    async fn log_diagnostics(&self) -> Result<(), MatterError> {
        loop {
            Timer::after_secs(DIAG_INTERVAL_SECS).await;

            let mut ap: sys::wifi_ap_record_t = unsafe { core::mem::zeroed() };
            let rssi = (unsafe { sys::esp_wifi_sta_get_ap_info(&mut ap) } == sys::ESP_OK)
                .then_some(ap.rssi);
            self.stack.matter().with_state(|state| {
                info!(
                    "diag: uptime={}s heap_free={} heap_min={} heap_largest={} fabrics={} sessions={} rssi={:?}",
                    unsafe { sys::esp_timer_get_time() } / 1_000_000,
                    unsafe { sys::esp_get_free_heap_size() },
                    unsafe { sys::esp_get_minimum_free_heap_size() },
                    unsafe { sys::heap_caps_get_largest_free_block(sys::MALLOC_CAP_8BIT) },
                    state.fabrics.iter().count(),
                    state.sessions.iter().count(),
                    rssi,
                );
                for s in state.sessions.iter().filter(|s| s.is_encrypted()) {
                    info!(
                        "diag:   session fab={} peer={:?} addr={}",
                        s.get_local_fabric_idx(),
                        s.get_peer_node_id(),
                        s.get_peer_addr(),
                    );
                }
            });
        }
    }

    async fn monitor_connectivity<N>(&self, netif: &N) -> Result<(), MatterError>
    where
        N: NetifDiag + NetChangeNotif,
    {
        loop {
            let mut link_ready = false;
            netif.netifs(&mut |info| {
                link_ready |=
                    info.operational && info.ipv6_addrs.iter().any(|addr| !addr.is_unspecified());
                Ok(())
            })?;
            NetStatus::from_link(self.stack.matter().has_fabrics(), link_ready)
                .store(&self.net_status);

            // Fabric changes need not emit network events.
            select(netif.wait_changed(), Timer::after_millis(250)).await;
        }
    }

    async fn run_maintenance(&self) -> Result<(), MatterError> {
        // Keep the existing SNTP policy; smooth sync can still step by more than 35 minutes.
        let clock_was_bogus = Utc::now().year() < 2026;
        let synced = if !self.had_fabric_at_start && clock_was_bogus {
            warn!("No fabric at startup and clock unsynced; deferring SNTP to the next boot");
            false
        } else {
            info!("Starting SNTP time sync (smooth mode)");
            let conf = SntpConf {
                sync_mode: SyncMode::Smooth,
                ..Default::default()
            };
            match EspSntp::new(&conf) {
                Ok(sntp) => {
                    let mut waited_secs = 0u32;
                    while sntp.get_sync_status() != SyncStatus::Completed {
                        if waited_secs >= 120 {
                            warn!("SNTP did not sync within 120s; using fallback reboot timer");
                            break;
                        }
                        Timer::after_secs(1).await;
                        waited_secs += 1;
                    }
                    core::mem::forget(sntp);
                    Utc::now().year() >= 2025
                }
                Err(e) => {
                    error!("Failed to start SNTP: {e:?}");
                    false
                }
            }
        };

        if synced && clock_was_bogus {
            info!(
                "Clock stepped forward; resetting Matter transport to clear stale session timers"
            );
            for attempts in 1..=10 {
                match self.stack.matter().reset_transport() {
                    Ok(()) => {
                        info!("Matter transport reset after clock step");
                        break;
                    }
                    Err(e) => {
                        if attempts == 10 {
                            warn!("Could not reset transport after {attempts} attempts: {e:?}");
                            break;
                        }
                        Timer::after_millis(200).await;
                    }
                }
            }
        }

        let reboot_point = if synced {
            let next_date = Utc::now()
                .with_timezone(&Los_Angeles)
                .date_naive()
                .succ_opt()
                .expect("date has a successor");
            let next_midnight = next_date.and_hms_opt(0, 0, 0).expect("00:00:00 is valid");
            let point = Los_Angeles
                .from_local_datetime(&next_midnight)
                .single()
                .expect("midnight is unambiguous")
                .with_timezone(&Utc);
            info!("Reboot at {next_midnight} Pacific ({point} UTC)");
            point
        } else {
            warn!("Clock unsynced; falling back to reboot ~24h from now");
            Utc::now() + chrono::Duration::hours(24)
        };

        while Utc::now() < reboot_point {
            Timer::after_secs(60).await;
        }
        info!("Scheduled time reached, rebooting...");
        unsafe {
            sys::esp_restart();
        }
    }
}

const SOLENOID_ENDPOINT_ID: u16 = 1;

const NODE: Node = Node {
    endpoints: &[
        root_endpoint!(wifi),
        Endpoint::new(
            SOLENOID_ENDPOINT_ID,
            devices!(DeviceType {
                dtype: 0xA,
                drev: 3
            }),
            clusters!(SolenoidHandler::CLUSTER, DescHandler::CLUSTER),
        ),
    ],
};

/// Frees the BLE controller's RAM for the WiFi/LWIP TX path and gives WiFi the
/// radio. The GATT task stays parked: it only touches BLE on indications,
/// which need a BTP session.
fn shutdown_ble() {
    info!("Shutting down BLE");
    unsafe {
        sys::esp_bluedroid_disable();
        sys::esp_bluedroid_deinit();
        sys::esp_bt_controller_disable();
        sys::esp_bt_controller_deinit();
        sys::esp_coex_preference_set(sys::esp_coex_prefer_t_ESP_COEX_PREFER_WIFI);
    }
    info!("BLE shut down");
}
