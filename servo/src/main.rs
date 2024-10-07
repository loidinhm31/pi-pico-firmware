#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::clocks;
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_time::Timer;
use fixed::types::extra::U4;
use fixed::FixedU16;

#[allow(unused_imports)]
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Configure and initialize PWM module
    // Integer part
    let int_part: u16 = 100;
    // Fractional part (3/16 corresponds to 3 in the 4-bit fractional representation)
    let frac_part: u16 = 0;
    let value_bits: u16 = (int_part << 4) | frac_part;
    let divider: FixedU16<U4> = FixedU16::from_bits(value_bits);

    let system_clock_hz = clocks::clk_sys_freq();
    let period = 20_u32;
    let pwm_freq_hz = 1000 / period; // Desired PWM frequency is 50 Hz (Period = 20 ms) f = 1/t
    let pwd_divider = 100;
    let top_value = calculate_pwm_top(system_clock_hz, pwm_freq_hz, pwd_divider);

    // Single Arm has 3 joints: init Config & Pwm for first 2 joints
    let mut c0: PwmConfig = Default::default();
    c0.divider = divider;
    c0.top = top_value as u16;
    let mut pwm0 = Pwm::new_output_b(p.PWM_CH5, p.PIN_27, c0.clone());
    // Init Pio Servo for 3rd join

    // Set duty cycle to make the servo move (pulse width for servos)
    let min_limit = ((top_value * 500) / (period * 1000)) as u16; // ~2.5% duty (500 µs / 20 ms)
    let max_limit = ((top_value * 2400) / (period * 1000)) as u16; // ~12% duty (2400 µs / 20 ms)

    // Infinite loop, rotating the servo left or right
    // SG90-HV datasheet
    // Right Max: 25000 *  2.5% = 625
    // Stop     : 25000 *  7.5% = 1875
    // Left  Max: 25000 * 12.0% = 3000
    loop {
        for angle in 0..=180 {
            c0.compare_b = map_angle_to_duty(angle, 0, 180, min_limit as u32, max_limit as u32) as u16;
            pwm0.set_config(&c0);
            Timer::after_millis(12).await;
        }
        //
        for angle in (0..=180).rev() {
            c0.compare_b = map_angle_to_duty(angle, 0, 180, min_limit as u32, max_limit as u32) as u16;
            pwm0.set_config(&c0);
            Timer::after_millis(12).await;
        }
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