#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use core::str::from_utf8;

use cyw43::{Control, JoinOptions};
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, IpAddress, IpEndpoint, Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use rand::RngCore;
use rust_mqtt::client::client::MqttClient;
use rust_mqtt::client::client_config::ClientConfig;
use rust_mqtt::utils::rng_generator::CountingRng;
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;
use core::sync::atomic::{AtomicBool, Ordering};
use {defmt_rtt as _, panic_probe as _};


bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[derive(Serialize, Deserialize)]
struct ArmPosition {
    angle: f32,
    control_type: heapless::String<3>,
}

type ArmPositions = heapless::Vec<ArmPosition, 16>;

#[derive(Format)]
enum JsonError {
    ParseFailed,
}
static CONTROL: StaticCell<Mutex<CriticalSectionRawMutex, Control<'static>>> = StaticCell::new();
static STACK: StaticCell<Mutex<CriticalSectionRawMutex, Stack<'static>>> = StaticCell::new();

static WIFI_CONNECTED: AtomicBool = AtomicBool::new(false);
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Program started");

    let p = embassy_rp::init(Default::default());
    let mut rng = RoscRng;

    let fw = include_bytes!("../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../cyw43-firmware/43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, control, runner) = cyw43::new(state, pwr, spi, fw).await;

    let control = CONTROL.init(Mutex::new(control));
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.lock().await.init(clm).await;
    control.lock().await
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let wifi_ssid = env!("WIFI_SSID");
    let wifi_password = env!("WIFI_PASSWORD");
    let mqtt_username = env!("MQTT_USERNAME");
    let mqtt_password = env!("MQTT_PASSWORD");
    let mqtt_server_name = env!("MQTT_SERVER_NAME");
    let mqtt_server_port = env!("MQTT_SERVER_PORT");

    let config = Config::dhcpv4(Default::default());

    // Generate random seed
    let seed = rng.next_u64();

    // Init network stack
    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    );
    let stack = STACK.init(Mutex::new(stack));

    unwrap!(spawner.spawn(net_task(runner)));
    loop {
        match control.lock().await
            .join(wifi_ssid, JoinOptions::new(wifi_password.as_bytes()))
            .await
        {
            Ok(_) => break,
            Err(err) => {
                info!("join failed with status={}", err.status);
            }
        }
    }

    // Wait for DHCP, not necessary when using static IP
    info!("waiting for DHCP...");
    let start = Instant::now().as_millis();
    loop {
        let elapsed = Instant::now().as_millis() - start;
        if elapsed > 10000 {
            core::panic!("Couldn't get network up after 10 seconds");
        } else if stack.lock().await.is_config_up() {
            info!("Network stack config completed after about {} ms", elapsed);
            break;
        } else {
            Timer::after_millis(10).await;
        }
    }

    let mac_addr = stack.lock().await.hardware_address();
    info!("Hardware configured. MAC Address is {}", mac_addr);

    match stack.lock().await.config_v4() {
        Some(a) => info!("IP Address appears to be: {}", a.address),
        None => core::panic!("DHCP completed but no IP address was assigned!"),
    }

    let server_address = stack.lock().await
        .dns_query(mqtt_server_name, embassy_net::dns::DnsQueryType::A)
        .await
        .unwrap();

    let dest: IpAddress = server_address.first().unwrap().clone();
    info!(
        "Our server named {} resolved to the address {}",
        mqtt_server_name, dest
    );

    unwrap!(spawner.spawn(wifi_task(control, stack, wifi_ssid, wifi_password)));
    // Wait for initial WiFi connection
    while !WIFI_CONNECTED.load(Ordering::Relaxed) {
        Timer::after(Duration::from_millis(100)).await;
    }


    // And now we can use it!
    let comm_port = 9932;
    let server_address = stack.lock().await
        .dns_query(mqtt_server_name, embassy_net::dns::DnsQueryType::A)
        .await
        .unwrap();

    let dest: IpAddress = server_address.first().unwrap().clone();
    info!(
        "Our server named {} resolved to the address {}",
        mqtt_server_name, dest
    );

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut msg_buffer = [0; 128];

    let mut socket = TcpSocket::new(*stack.lock().await, &mut rx_buffer, &mut tx_buffer);
    socket
        .connect(IpEndpoint::new(dest, comm_port))
        .await
        .unwrap();

    let tx_size = socket.write("test".as_bytes()).await.unwrap();
    info!("Wrote {} byes to the server", tx_size);
    let rx_size = socket.read(&mut msg_buffer).await.unwrap();
    let response = from_utf8(&msg_buffer[..rx_size]).unwrap();
    info!("Server replied with {}", response);

    socket.close();

    // MQTT setup
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut mqtt_socket = TcpSocket::new(*stack.lock().await, &mut rx_buffer, &mut tx_buffer);
    info!("Connecting to MQTT broker at {}:{}", dest, mqtt_server_port);
    match mqtt_socket
        .connect(IpEndpoint::new(dest, mqtt_server_port.parse().unwrap()))
        .await
    {
        Ok(()) => info!("Connected to MQTT broker"),
        Err(e) => {
            error!("Failed to connect to MQTT broker: {:?}", e);
        }
    }

    let mut mqtt_rx_buffer = [0; 1024];
    let mut mqtt_tx_buffer = [0; 1024];

    let mut config = ClientConfig::new(
        rust_mqtt::client::client_config::MqttVersion::MQTTv5,
        CountingRng(20000),
    );
    config.add_client_id("pico_w");
    config.add_username(mqtt_username);
    config.add_password(mqtt_password);

    let mut mqtt_client = MqttClient::<_, 5, _>::new(
        mqtt_socket,
        &mut mqtt_rx_buffer,
        1024,
        &mut mqtt_tx_buffer,
        1024,
        config,
    );

    // Set up MQTT connection with authentication
    info!("Connecting to MQTT broker...");
    match mqtt_client.connect_to_broker().await {
        Ok(()) => info!("Connected to MQTT broker successfully"),
        Err(e) => {
            error!("Failed to connect to MQTT broker: {:?}", e);
        }
    }

    info!("Subscribing to topic: armbot/position");
    match mqtt_client.subscribe_to_topic("armbot/position").await {
        Ok(()) => info!("Subscribed to topic successfully"),
        Err(e) => {
            error!("Failed to subscribe to topic: {:?}", e);
        }
    }


    loop {
        control.lock().await.gpio_set(0, false).await;
        Timer::after(Duration::from_secs(1)).await;
        match mqtt_client.receive_message().await {
            Ok((topic, payload)) => {
                if topic == "armbot/position" {
                    control.lock().await.gpio_set(0, true).await;
                    Timer::after(Duration::from_millis(100)).await;

                    // Parse the JSON payload
                    match serde_json_core::from_slice::<ArmPositions>(payload) {
                        Ok((positions, _)) => {
                            for (i, position) in positions.iter().enumerate() {
                                info!("Position {}: {} - {}",
                                    i, position.control_type, position.angle);

                                // Use the positions array as needed
                                let mut json_buffer = [0u8; 256]; // Increased buffer size to accommodate the array
                                let json = serde_json_core::to_slice(&position, &mut json_buffer);
                                match json {
                                    Ok(len) => {
                                        // Send each position as a separate message
                                        match mqtt_client.send_message(
                                            "armbot/test",
                                            &json_buffer[..len],
                                            rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1,
                                            false,
                                        ).await {
                                            Ok(()) => info!("Sent position {}", i),
                                            Err(_e) => error!("Failed to send position {}: {:?}", i, _e),
                                        }
                                    }
                                    Err(_e) => error!("Failed to serialize position {}: {:?}", i, JsonError::ParseFailed),
                                }
                            }
                        }
                        Err(_) => error!("Failed to parse JSON: {:?}", JsonError::ParseFailed),
                    }
                }
            }
            Err(e) => {
                error!("MQTT error: {:?}", e);
                // Consider implementing a reconnection strategy here
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn wifi_task(
    control: &'static Mutex<CriticalSectionRawMutex, Control<'static>>,
    stack: &'static Mutex<CriticalSectionRawMutex, Stack<'static>>,
    ssid: &'static str,
    password: &'static str,
) {
    const CHECK_INTERVAL_CONNECTED: Duration = Duration::from_secs(120);  // Check every ? seconds when connected
    const CHECK_INTERVAL_DISCONNECTED: Duration = Duration::from_secs(5);  // Check every 5 seconds when disconnected
    const INITIAL_CHECK_INTERVAL: Duration = Duration::from_millis(100);  // Initial rapid checks

    let mut check_interval = INITIAL_CHECK_INTERVAL;

    loop {
        {
            let mut control_guard = control.lock().await;
            control_guard.gpio_set(0, true).await;
        }

        let is_connected = {
            let stack_guard = stack.lock().await;
            stack_guard.is_link_up()
        };

        if !is_connected {
            info!("WiFi disconnected. Attempting to reconnect...");
            let mut control_guard = control.lock().await;
            match control_guard.join(ssid, JoinOptions::new(password.as_bytes())).await {
                Ok(_) => {
                    info!("WiFi connected successfully");
                    WIFI_CONNECTED.store(true, Ordering::Relaxed);
                    check_interval = CHECK_INTERVAL_CONNECTED;
                },
                Err(err) => {
                    error!("Failed to connect to WiFi: {}", err.status);
                    WIFI_CONNECTED.store(false, Ordering::Relaxed);
                    check_interval = CHECK_INTERVAL_DISCONNECTED;
                }
            }
        } else {
            if !WIFI_CONNECTED.load(Ordering::Relaxed) {
                info!("WiFi connection restored");
            }
            WIFI_CONNECTED.store(true, Ordering::Relaxed);
            check_interval = CHECK_INTERVAL_CONNECTED;
        }

        {
            let mut control_guard = control.lock().await;
            control_guard.gpio_set(0, false).await;
        }

        // Sleep for the determined interval
        Timer::after(check_interval).await;
    }
}