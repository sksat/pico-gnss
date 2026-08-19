//! Which edge of the receiver's 1PPS marks the second, and how to point the capture at it.
//!
//! [`rp_pps::pps_capture_program`] watches for a rising edge. The receiver on this bench idles high
//! and pulses low — its board documents `1PPS 出力 : C-MOS ロジック (3.3V) レベル,
//! パルス幅 :100mS (アクティブ Low)`, and the 1PPS passes through one gate of the 74HC04 on that
//! board — so the rising edge is the *end* of the pulse, 100 ms past the second.
//!
//! Nothing downstream notices. Every interval is still exactly one second, the frequency estimate
//! is unaffected, and the output still locks to the input within nanoseconds. The clock is simply
//! 100 ms late, and only a comparison against an outside clock shows it: measured through a real
//! NTP exchange, this server read 100.23 ms slow (sd 0.55 ms, n=299).

use embassy_rp::pac;
use embassy_time::{Duration, Instant, Timer};
use rp_pps::{PolarityProbe, PpsPolarity};

/// The polarity of the 1PPS this firmware is built for.
///
/// Configured rather than measured. A boot-time probe can only report what the pin was doing
/// during its window, and a receiver without a fix drives no pulse at all — the AE-GNSS-EXTANT
/// datasheet gives 1PPS as `3D_Fix 時のみ出力`. Reading a resting pin gives an answer that looks
/// like a measurement and is a guess. On a board whose wiring is known, the wiring is the better
/// source, and it costs no boot time.
///
/// Change this for a different receiver or carrier board, and use [`probe_polarity`] to find out
/// what to change it to.
pub const PPS_POLARITY: PpsPolarity = PpsPolarity::ActiveLow;

/// Point the capture at the marking edge, using [`PPS_POLARITY`].
///
/// Call after [`rp_pps::embassy::PpsCapture`] (or the timed variant) has taken the pin: that
/// assignment rewrites the register the inversion lives in.
pub fn configure_capture(pin: usize) -> PpsPolarity {
    rp_pps::embassy::set_capture_polarity(pin, PPS_POLARITY);
    defmt::info!(
        "PPS on GP{}: configured {}",
        pin,
        match PPS_POLARITY {
            PpsPolarity::ActiveLow => "active low, capturing the falling edge",
            PpsPolarity::ActiveHigh => "active high, capturing the rising edge",
        }
    );
    PPS_POLARITY
}

/// How long to watch the pin: long enough to hold a whole second whichever level is the short one.
const PROBE_WINDOW: Duration = Duration::from_millis(1_500);
const PROBE_STEP: Duration = Duration::from_micros(500);

/// Watch `pin` for a second and a half and report the polarity of the pulse it carries, or `None`
/// if it carried none.
///
/// A bring-up tool for an unknown receiver or carrier board, not part of the boot path: it costs
/// the window, and it can only answer while a pulse is actually running. Read the answer from the
/// log and put it in [`PPS_POLARITY`].
///
/// Sample **before** [`configure_capture`]: the inversion it applies is visible to SIO as well, so
/// afterwards this would read the inverted waveform and conclude the opposite.
pub async fn probe_polarity(pin: usize) -> Option<PpsPolarity> {
    let mut probe = PolarityProbe::new();
    let deadline = Instant::now() + PROBE_WINDOW;
    while Instant::now() < deadline {
        probe.sample(pac::SIO.gpio_in(0).read() & (1 << pin) != 0);
        Timer::after(PROBE_STEP).await;
    }

    let duty = probe.duty_percent().unwrap_or(0);
    if !probe.saw_pulse() {
        defmt::warn!(
            "PPS on GP{}: {}% high, no pulse — nothing to conclude",
            pin,
            duty
        );
        return None;
    }
    let polarity = probe.polarity();
    defmt::info!(
        "PPS on GP{}: {}% high — {}",
        pin,
        duty,
        match polarity {
            Some(PpsPolarity::ActiveLow) => "active low",
            _ => "active high",
        }
    );
    polarity
}
