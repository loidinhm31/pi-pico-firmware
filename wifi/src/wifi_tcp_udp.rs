#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use core::str::from_utf8;

use cyw43::JoinOptions;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Config, IpAddress, IpEndpoint, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write;
use rand::RngCore;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Program started");

    let p = embassy_rp::init(Default::default());
    let mut rng = RoscRng;

    let fw = include_bytes!("../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --binary-format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --binary-format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

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

    let config = Config::dhcpv4(Default::default());
    // let config = Config::ipv4_static(embassy_net::StaticConfigV4 {
    //     address: embassy_net::Ipv4Cidr::new(embassy_net::Ipv4Address::new(192, 168, 1, 249), 16),
    //     dns_servers: heapless::Vec::new(),
    //     gateway: None,
    // });

    // Generate random seed
    let seed = rng.next_u64();

    // Init network stack
    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(net_device, config, RESOURCES.init(StackResources::new()), seed);

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
    info!("waiting for DHCP...");
    let start = Instant::now().as_millis();
    loop {
        let elapsed = Instant::now().as_millis() - start;
        if elapsed > 10000 {
            core::panic!("Couldn't get network up after 10 seconds");
        } else if stack.is_config_up() {
            info!("Network stack config completed after about {} ms", elapsed);
            break;
        } else {
            Timer::after_millis(10).await;
        }
    }
    // while !stack.is_config_up() {
    //     Timer::after_millis(100).await;
    // }

    let mac_addr = stack.hardware_address();
    info!("Hardware configured. MAC Address is {}", mac_addr);

    match stack.config_v4() {
        Some(a) => info!("IP Address appears to be: {}", a.address),
        None => core::panic!("DHCP completed but no IP address was assigned!"),
    }

    // And now we can use it!
    let server_name = "192.168.1.114";
    let comm_port = 9932;
    let server_address = stack
        .dns_query(server_name, embassy_net::dns::DnsQueryType::A)
        .await
        .unwrap();

    let dest: IpAddress = server_address.first().unwrap().clone();
    info!(
        "Our server named {} resolved to the address {}",
        server_name, dest
    );

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut msg_buffer = [0; 128];

    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
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

    // Reuse the earlier msg_buffer since we're done with the TCP part
    let mut udp_rx_meta = [PacketMetadata::EMPTY; 16];
    let mut udp_rx_buffer = [0; 1024];
    let mut udp_tx_meta = [PacketMetadata::EMPTY; 16];
    let mut udp_tx_buffer = [0; 1024];

    let mut udp_socket = UdpSocket::new(
        stack,
        &mut udp_rx_meta,
        &mut udp_rx_buffer,
        &mut udp_tx_meta,
        &mut udp_tx_buffer,
    );

    udp_socket.bind(0).unwrap();

    loop {
        // test_tcp(&mut socket, &mut control, &mut msg_buffer).await;
        test_udp(&mut udp_socket, &mut control, &mut msg_buffer, &dest, comm_port).await;
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


async fn test_tcp(socket: &mut TcpSocket<'_>, control: &mut cyw43::Control<'_>, msg_buffer: &mut [u8]) {
    socket.set_timeout(Some(Duration::from_secs(10)));

    control.gpio_set(0, false).await;
    info!("Listening on TCP:1234...");
    if let Err(e) = socket.accept(1234).await {
        warn!("accept error: {:?}", e);
    }

    info!("Received connection from {:?}", socket.remote_endpoint());
    control.gpio_set(0, true).await;

    loop {
        let n = match socket.read(msg_buffer).await {
            Ok(0) => {
                warn!("read EOF");
                break;
            }
            Ok(n) => n,
            Err(e) => {
                warn!("read error: {:?}", e);
                break;
            }
        };

        info!("rxd {}", from_utf8(&msg_buffer[..n]).unwrap());

        match socket.write_all(&msg_buffer[..n]).await {
            Ok(()) => {}
            Err(e) => {
                warn!("write error: {:?}", e);
                break;
            }
        };
    }
}

async fn test_udp(udp_socket: &mut UdpSocket<'_>, control: &mut cyw43::Control<'_>, msg_buffer: &mut [u8], dest: &IpAddress, comm_port: u16) {
    info!("external LED on, onboard LED off!");
    control.gpio_set(0, false).await;
    info!("sending UDP packet");
    udp_socket
        .send_to("test".as_bytes(), IpEndpoint::new(*dest, comm_port))
        .await
        .unwrap();
    Timer::after(Duration::from_secs(1)).await;

    info!("external LED off, onboard LED on!");
    control.gpio_set(0, true).await;
    if udp_socket.may_recv() {
        let (rx_size, from_addr) = udp_socket.recv_from(msg_buffer).await.unwrap();
        let response = from_utf8(&msg_buffer[..rx_size]).unwrap();
        info!("Server replied with {} from {}", response, from_addr);
    }
    Timer::after(Duration::from_secs(1)).await;
}