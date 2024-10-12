#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use core::mem::MaybeUninit;
use core::str;
use core::str::from_utf8;

use core::sync::atomic::{AtomicBool, Ordering};
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
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use rand::RngCore;
use rust_mqtt::client::client::MqttClient;
use rust_mqtt::client::client_config::ClientConfig;
use rust_mqtt::utils::rng_generator::CountingRng;
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;
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
static CONTROL: StaticCell<Control<'static>> = StaticCell::new();
static STACK: StaticCell<Stack<'static>> = StaticCell::new();

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
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let wifi_ssid = env!("WIFI_SSID");
    let wifi_password = env!("WIFI_PASSWORD");
    let mqtt_username = env!("MQTT_USERNAME");
    let mqtt_password = env!("MQTT_PASSWORD");
    let mqtt_server_name = env!("MQTT_SERVER_NAME");
    let mqtt_server_port: u16 = env!("MQTT_SERVER_PORT").parse().unwrap();

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

    unwrap!(spawner.spawn(net_task(runner)));
    loop {
        match control
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
    info!("Waiting for DHCP...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    info!("DHCP is now up!");
    info!("waiting for link up...");
    while !stack.is_link_up() {
        Timer::after_millis(500).await;
    }
    info!("Link is up!");

    info!("waiting for stack to be up...");
    stack.wait_config_up().await;
    info!("Stack is up!");

    // Start use Wi-Fi
    let mac_addr = stack.hardware_address();
    info!("Hardware configured. MAC Address is {}", mac_addr);

    match stack.config_v4() {
        Some(a) => info!("IP Address appears to be: {}", a.address),
        None => core::panic!("DHCP completed but no IP address was assigned!"),
    }

    let stack = STACK.init(stack);
    let control = CONTROL.init(control);

    unwrap!(spawner.spawn(wifi_task(control, stack, wifi_ssid, wifi_password)));
    unwrap!(spawner.spawn(mqtt_task(stack, mqtt_server_name, mqtt_server_port, mqtt_username, mqtt_password)));

    let server_address = stack
        .dns_query(mqtt_server_name, embassy_net::dns::DnsQueryType::A)
        .await
        .unwrap();

    let dest: IpAddress = server_address.first().unwrap().clone();
    info!(
        "Our server named {} resolved to the address {}",
        mqtt_server_name, dest
    );

    let comm_port = 9932;
    let server_address = stack
        .dns_query(mqtt_server_name, embassy_net::dns::DnsQueryType::A)
        .await
        .unwrap();

    let dest: IpAddress = server_address.first().unwrap().clone();
    info!(
        "Our server named {} resolved to the address {}",
        mqtt_server_name, dest
    );

    loop {
        Timer::after(Duration::from_secs(1)).await;
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
    control: &'static mut Control<'static>,
    stack: &'static Stack<'static>,
    ssid: &'static str,
    password: &'static str,
) {
    const CHECK_INTERVAL_CONNECTED: Duration = Duration::from_secs(10);  // Check every ? seconds when connected
    const CHECK_INTERVAL_DISCONNECTED: Duration = Duration::from_secs(5);  // Check every 5 seconds when disconnected
    const INITIAL_CHECK_INTERVAL: Duration = Duration::from_millis(100);  // Initial rapid checks

    let mut check_interval = INITIAL_CHECK_INTERVAL;
    info!("Running WiFi task...");
    loop {
        control.gpio_set(0, true).await;

        info!("Checking WiFi connection...");

        let is_connected = stack.is_link_up();

        if !is_connected {
            info!("WiFi disconnected. Attempting to reconnect...");
            Timer::after(Duration::from_millis(5000)).await;
        } else {
            info!("WiFi connected...");

            Timer::after(Duration::from_millis(1000)).await;
            check_interval = CHECK_INTERVAL_CONNECTED;
        }

        control.gpio_set(0, false).await;

        // Sleep for the determined interval
        Timer::after(check_interval).await;
    }
}

#[embassy_executor::task]
async fn mqtt_task(
    stack: &'static Stack<'static>,
    mqtt_server_name: &'static str,
    mqtt_server_port: u16,
    mqtt_username: &'static str,
    mqtt_password: &'static str,
) {
    const RECONNECT_INTERVAL: Duration = Duration::from_secs(5);
    const KEEP_ALIVE: u16 = 60;
    const MAX_RECONNECT_ATTEMPTS: u32 = 5;

    // Allocate buffers
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut mqtt_rx_buffer = [0; 1024];
    let mut mqtt_tx_buffer = [0; 1024];

    loop {
        let mut mqtt_socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);

        info!("Initializing MQTT connection...");
        // Resolve server address
        let server_address = match stack.dns_query(mqtt_server_name, embassy_net::dns::DnsQueryType::A).await {
            Ok(addresses) => addresses.first().cloned(),
            Err(e) => {
                error!("DNS resolution failed: {:?}", e);
                Timer::after(RECONNECT_INTERVAL).await;
                continue;
            }
        };

        let dest = match server_address {
            Some(addr) => addr,
            None => {
                error!("No IP address found for MQTT server");
                Timer::after(RECONNECT_INTERVAL).await;
                continue;
            }
        };

        info!("MQTT server resolved to {}", dest);
        info!("Connecting to MQTT broker at {}:{}", dest, mqtt_server_port);
        let _connect_result = mqtt_socket
            .connect(IpEndpoint::new(dest, mqtt_server_port))
            .await
            .unwrap();

        let mut config = ClientConfig::new(
            rust_mqtt::client::client_config::MqttVersion::MQTTv5,
            CountingRng(20000),
        );
        config.add_client_id("pico_w");
        config.add_username(mqtt_username);
        config.add_password(mqtt_password);
        config.keep_alive = KEEP_ALIVE;

        let mut mqtt_client = MqttClient::<_, 5, _>::new(
            mqtt_socket,
            &mut mqtt_rx_buffer,
            1024,
            &mut mqtt_tx_buffer,
            1024,
            config,
        );

        // Set up MQTT socket
        let mut reconnect_attempts = 0;
        while reconnect_attempts < MAX_RECONNECT_ATTEMPTS {
            // Connect to MQTT broker
            info!("Connecting to MQTT broker...");
            if let Err(e) = mqtt_client.connect_to_broker().await {
                error!("Failed to connect to MQTT broker: {:?}", e);
                reconnect_attempts += 1;
                Timer::after(RECONNECT_INTERVAL).await;
                continue;
            }
            info!("Connected to MQTT broker successfully");

            // Subscribe to topic
            info!("Subscribing to topic: armbot/position");
            if let Err(e) = mqtt_client.subscribe_to_topic("armbot/position").await {
                error!("Failed to subscribe to topic: {:?}", e);
                reconnect_attempts += 1;
                Timer::after(RECONNECT_INTERVAL).await;
                continue;
            }
            info!("Subscribed to topic successfully");

            // Reset reconnect attempts on successful connection
            reconnect_attempts = 0;

            // Main MQTT loop
            loop {
                match mqtt_client.receive_message().await {
                    Ok((topic, payload)) => {
                        if topic == "armbot/position" {
                            // Handle the received message
                            // control.lock().await.gpio_set(0, true).await;
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
                        break; // Break the inner loop to reconnect
                    }
                }

                Timer::after(Duration::from_millis(10)).await;
            }

            // If we've reached here, there was an error, and we need to reconnect
            error!("MQTT connection lost. Reconnecting...");
            reconnect_attempts += 1;
            Timer::after(RECONNECT_INTERVAL).await;
        }
        error!("Max reconnection attempts reached. Waiting before trying again...");
        Timer::after(Duration::from_secs(KEEP_ALIVE as u64)).await;
    }
}