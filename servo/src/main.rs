#![no_std]
#![no_main]

use rp_pico as bsp;

use bsp::entry;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    prelude::*,
    sio::Sio,
    watchdog::Watchdog,
};
use cortex_m::prelude::_embedded_hal_PwmPin;
use defmt::*;

#[allow(unused_imports)]
use defmt_rtt as _;
#[allow(unused_imports)]
use panic_probe as _;


#[entry]
fn main() -> ! {
    info!("Program start");

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let system_clock_hz = clocks.system_clock.freq().to_Hz();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, system_clock_hz);

    // Init PWMs
    let mut pwm_slices = bsp::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM5
    let pwm = &mut pwm_slices.pwm5;
    pwm.enable();

    let period = 20_u32;
    let pwm_freq_hz = 1000 / period; // Desired PWM frequency is 50 Hz (Period = 20 ms) f = 1/t
    let pwd_divider = 100;
    let top_value = calculate_pwm_top(system_clock_hz, pwm_freq_hz, pwd_divider);

    pwm.set_top(top_value as u16);
    pwm.set_div_int(pwd_divider as u8);
    pwm.set_div_frac(0);

    // Output channel B on PWMt to GPIO 27
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio27);

    // Set duty cycle to make the servo move (pulse width for servos)
    let min_limit = ((top_value * 500) / (period * 1000)) as u16; // ~2.5% duty (500 µs / 20 ms)
    let max_limit = ((top_value * 2400) / (period * 1000)) as u16; // ~12% duty (2400 µs / 20 ms)

    // Infinite loop, rotating the servo left or right
    // SG90-HV datasheet
    // Right Max: 25000 *  5.0% = 625
    // Stop     : 25000 *  7.5% = 1875
    // Left  Max: 25000 * 12.0% = 3000
    loop {
        for angle in 0..=180 {
            if angle % 5 == 0 {
                channel.set_duty(map_angle_to_duty(angle, 0, 180, min_limit as u32, max_limit as u32) as u16);
                delay.delay_ms(5);
            }
        }

        for angle in (0..=180).rev() {
            if angle % 5 == 0 {
                channel.set_duty(map_angle_to_duty(angle, 0, 180, min_limit as u32, max_limit as u32) as u16);
                delay.delay_ms(5);
            }
        }
        delay.delay_ms(500);
    }
}

/// Calculate the TOP value for the given system clock frequency and desired PWM frequency.
///
/// # Arguments
///
/// * `system_clock_hz` - The system clock frequency in Hz (e.g., 125,000,000 Hz).
/// * `pwm_freq_hz` - The desired PWM frequency in Hz (e.g., 50 Hz).
///
/// # Returns
///
/// The TOP value for the PWM timer.
fn calculate_pwm_top(system_clock_hz: u32, pwm_freq_hz: u32, pwm_divider: u32) -> u32 {
    system_clock_hz / (pwm_freq_hz * pwm_divider)
}

// Function that maps one range to another
fn map_angle_to_duty(deg: u32, in_min: u32, in_max: u32, out_min: u32, out_max: u32) -> u32 {
    if deg <= 0 {
        out_min
    } else if deg >= 180 {
        out_max
    } else {
        out_min + ((deg - in_min) * (out_max - out_min) / (in_max - in_min))
    }
}

