use core::pin::pin;

use std::sync::Arc;

use doorbell2::door_lock::{self, ClusterAsyncHandler};
use doorbell2::SolenoidHandler;
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
use static_cell::StaticCell;

use std::sync::atomic::{AtomicBool, Ordering};

const STACK_SIZE: usize = 36 * 1024;
const BLUETOOTH_STACK_SIZE: usize = 20 * 1024;

/// WebSocket server URI to connect to for external unlock commands
const WS_SERVER_URI: &str = "ws://192.168.68.1:8080";

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
    let stack = MATTER_STACK
        .uninit()
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
    // Using `pin!` is completely optional, but reduces the size of the final future
    let matter = pin!(stack.run_coex(
        // The Matter stack needs the Wifi/BLE modem peripheral
        EspMatterWifi::new_with_builtin_mdns(peripherals.modem, sysloop, timers, nvs, stack),
        // The Matter stack needs a persister to store its state
        &persist,
        // The crypto provider
        &crypto,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // Tune WiFi for stability with BLE coex, then start WebSocket client
        WifiStabilityTask { stack, unlock_request },
    ));

    let mut led = PinDriver::output(peripherals.pins.gpio23).expect("GPIO 23 init");
    led.set_high().expect("LED high");

    // Run Matter
    matter.await?;

    Ok(())
}

/// The Matter stack is allocated statically to avoid
/// program stack blowups.
/// It is also a mandatory requirement when the `WifiBle` stack variation is used.
static MATTER_STACK: StaticCell<EspWifiMatterStack<BLUETOOTH_STACK_SIZE, ()>> = StaticCell::new();

/// Tunes WiFi for stability when coexisting with BLE on ESP32's shared radio.
/// Disables WiFi PS immediately, then polls for commissioning completion
/// before shutting down BLE to free the shared radio.
struct WifiStabilityTask {
    stack: &'static EspWifiMatterStack<'static, BLUETOOTH_STACK_SIZE, ()>,
    unlock_request: Arc<AtomicBool>,
}

impl UserTask for WifiStabilityTask {
    async fn run<S, N>(&mut self, _net_stack: S, netif: N) -> Result<(), rs_matter::error::Error>
    where
        S: nal::NetStack,
        N: rs_matter::dm::clusters::gen_diag::NetifDiag + rs_matter::dm::networks::NetChangeNotif,
    {
        unsafe {
            info!("Disabling WiFi power save");
            esp_idf_svc::sys::esp_wifi_set_ps(esp_idf_svc::sys::wifi_ps_type_t_WIFI_PS_NONE);
        }

        let commissioned = self.stack.matter().is_commissioned();
        info!("Commissioned state at UserTask start: {}", commissioned);

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
            netif.wait_changed().await;
        }

        info!("Shutting down BLE to stabilize WiFi");
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

        // Start WebSocket client to listen for external unlock commands
        info!("Connecting to WebSocket server: {}", WS_SERVER_URI);
        let unlock = self.unlock_request.clone();
        let _ws_client = esp_idf_svc::ws::client::EspWebSocketClient::new(
            WS_SERVER_URI,
            &esp_idf_svc::ws::client::EspWebSocketClientConfig::default(),
            core::time::Duration::from_secs(10),
            move |event| {
                if let Ok(event) = event {
                    match event.event_type {
                        esp_idf_svc::ws::client::WebSocketEventType::Connected => {
                            info!("WebSocket connected");
                        }
                        esp_idf_svc::ws::client::WebSocketEventType::Disconnected => {
                            warn!("WebSocket disconnected");
                        }
                        esp_idf_svc::ws::client::WebSocketEventType::Text(text) => {
                            info!("WebSocket received: {}", text);
                            if text == "open" {
                                info!("WebSocket: unlock command received");
                                unlock.store(true, Ordering::Relaxed);
                            }
                        }
                        _ => {}
                    }
                }
            },
        );

        match _ws_client {
            Ok(_client) => {
                info!("WebSocket client started");
                // Keep the client alive by holding it in scope while we await forever
                core::future::pending().await
            }
            Err(e) => {
                error!("Failed to start WebSocket client: {:?}", e);
                core::future::pending().await
            }
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
