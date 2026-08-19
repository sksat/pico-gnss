//! Point the PIO capture at the edge of the receiver's 1PPS that actually marks the second.
//!
//! [`rp_pps::pps_capture_program`] watches for a rising edge. The receiver on this bench idles
//! high and pulses low — its board documents `1PPS 出力 : C-MOS ロジック (3.3V) レベル,
//! パルス幅 :100mS (アクティブ Low)` — so the rising edge is the *end* of the pulse, 100 ms past
//! the second.
//!
//! Nothing downstream notices. Every interval is still exactly one second, the frequency estimate
//! is unaffected, and the disciplined output still locks to the input within nanoseconds. The
//! clock is simply 100 ms late, and only a comparison against an outside clock shows it: measured
//! through a real NTP exchange, this server read 100.23 ms slow (sd 0.55 ms, n=299).

use embassy_rp::pac;
use embassy_time::{Duration, Instant, Timer};
use rp_pps::{PolarityProbe, PpsPolarity};

/// Long enough to contain a whole second whichever level is the short one.
const WINDOW: Duration = Duration::from_millis(1_500);
const STEP: Duration = Duration::from_micros(500);

/// Watch `pin` for [`WINDOW`], then invert it into PIO if the pulse turns out to be the low
/// excursion. Returns what it decided.
///
/// Call this **after** the pin has been handed to PIO: that assignment rewrites the same control
/// register the inversion lives in, so an inversion set beforehand is silently dropped.
///
/// Measured rather than configured. `PMTK285` sets the pulse width at runtime and the setting
/// outlives a power cycle, so the duty is not something this build can know; a different module or
/// a rewired board would invalidate a constant without saying so.
pub async fn align_capture_edge(pin: usize) -> PpsPolarity {
    let mut probe = PolarityProbe::new();
    let deadline = Instant::now() + WINDOW;
    while Instant::now() < deadline {
        probe.sample(pac::SIO.gpio_in(0).read() & (1 << pin) != 0);
        Timer::after(STEP).await;
    }

    let polarity = probe.polarity().unwrap_or_default();
    if polarity == PpsPolarity::ActiveLow {
        pac::IO_BANK0
            .gpio(pin)
            .ctrl()
            .modify(|w| w.set_inover(pac::io::vals::Inover::INVERT));
    }
    defmt::info!(
        "PPS on GP{}: {}% high — {}",
        pin,
        probe.duty_percent().unwrap_or(0),
        match polarity {
            PpsPolarity::ActiveLow => "active low, inverting so the capture takes the falling edge",
            PpsPolarity::ActiveHigh => "active high, capturing the rising edge",
        }
    );
    polarity
}
