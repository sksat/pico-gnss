//! Do three counters in one PIO block agree, and does one of them work on a pin it does not own?
//!
//! Everything the next stage rests on is this assumption: that several free-running counters,
//! brought up together, form one timebase. It cannot be tested on a host — there is no counter —
//! and it fails quietly if it fails at all, as a fixed offset nobody can account for later.
//!
//! So: three state machines, one physical edge, and the differences between what they report.
//!
//! ```text
//!   GP2 (1PPS in) ──┬──► PIO0 SM0   claims the pin
//!                   ├──► PIO0 SM2   watches it by number
//!                   └──► PIO0 SM3   watches it by number
//! ```
//!
//! SM1 is left alone: it is the 1PPS output on the server, and a counter that is *not* in the
//! started set has to stay out of it.
//!
//! What to expect. The three counters run the same program at the same clock, so if they started
//! on the same cycle their values differ by a constant — and since they were loaded with the same
//! program at the same offset and see the same edge, that constant should be zero. A difference
//! that *drifts* means they are not one timebase, and the whole approach has to change.

#![no_std]
#![no_main]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::{Duration, Timer};

use rp_pps::embassy::{EventCapture, PpsCapture, set_capture_polarity, start_in_sync};
use rp_pps::{PpsPolarity, TickTimeline, pps_capture_program_wrap_balanced, ticks_to_ns};

use defmt_rtt as _;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/// The pin all three watch. GP2 is the receiver's 1PPS on this board, which is a clean 1 Hz edge.
const WATCHED_GPIO: u8 = 2;

/// The receiver's 1PPS is active low, so the capture program's rising edge has to be inverted into
/// PIO to land on the second. It does not matter for *this* test — all three see the same edge
/// either way — but leaving it wrong would make the reported times mean something else.
const PPS_POLARITY: PpsPolarity = PpsPolarity::ActiveLow;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let clk = clk_sys_freq();
    info!(
        "tsbase: three counters on GP{}, clk {} Hz",
        WATCHED_GPIO, clk
    );

    let Pio {
        mut common,
        sm0,
        sm2,
        sm3,
        ..
    } = Pio::new(p.PIO0, Irqs);
    let program = pps_capture_program_wrap_balanced();

    // One claims the pin, two watch it by number. All three stopped.
    let mut a = PpsCapture::<PIO0, 0>::new_stopped(&mut common, sm0, p.PIN_2, &program);
    set_capture_polarity(WATCHED_GPIO as usize, PPS_POLARITY);
    let mut b = EventCapture::<PIO0, 2>::new_stopped(
        &mut common,
        sm2,
        embassy_rp::pac::PIO0,
        WATCHED_GPIO,
        &program,
    );
    let mut c = EventCapture::<PIO0, 3>::new_stopped(
        &mut common,
        sm3,
        embassy_rp::pac::PIO0,
        WATCHED_GPIO,
        &program,
    );

    // One write: enable all three and restart their dividers on the same cycle.
    let mask = PpsCapture::<PIO0, 0>::sm_mask()
        | EventCapture::<PIO0, 2>::sm_mask()
        | EventCapture::<PIO0, 3>::sm_mask();
    start_in_sync(embassy_rp::pac::PIO0, mask);
    info!("started state machines {=u8:04b} together", mask);

    // `Common` must outlive `main`, or the pin goes back to nothing. See the note in the server.
    core::mem::forget(common);

    let mut edges: u32 = 0;
    let mut timeline = TickTimeline::new();
    let mut first_ba: Option<i64> = None;
    let mut first_ca: Option<i64> = None;

    loop {
        let ra = a.wait_edge().await;
        // The other two saw the same edge; give them a moment to have pushed it, then read.
        Timer::after(Duration::from_millis(1)).await;
        let (Some(rb), Some(rc)) = (b.try_read(), c.try_read()) else {
            warn!("edge {}: not every counter reported it", edges);
            b.drain();
            c.drain();
            edges = edges.wrapping_add(1);
            continue;
        };
        // Anything still queued would desynchronise the next reading.
        let extra = b.drain() + c.drain();
        if extra > 0 {
            warn!("edge {}: {} surplus captures dropped", edges, extra);
        }

        // Differences, not values: the counters start wherever they start.
        let ba = ra.wrapping_sub(rb) as i32 as i64;
        let ca = ra.wrapping_sub(rc) as i32 as i64;
        let ticks = timeline.observe(ra);
        edges = edges.wrapping_add(1);

        let base_ba = *first_ba.get_or_insert(ba);
        let base_ca = *first_ca.get_or_insert(ca);
        if edges <= 5 || edges.is_multiple_of(16) {
            info!(
                "edge {} t={=u64}s  b-a={} ticks ({} ns)  c-a={} ticks ({} ns)  drift b={} c={}",
                edges,
                ticks_to_ns(ticks, clk) / 1_000_000_000,
                ba,
                ticks_to_ns(ba.unsigned_abs(), clk) as i64 * ba.signum(),
                ca,
                ticks_to_ns(ca.unsigned_abs(), clk) as i64 * ca.signum(),
                ba - base_ba,
                ca - base_ca
            );
        }
        // A constant difference is a shared timebase. A growing one is not, and no amount of
        // averaging later will recover from it.
        if ba != base_ba || ca != base_ca {
            warn!(
                "edge {}: the counters have moved apart, b by {} ticks and c by {}",
                edges,
                ba - base_ba,
                ca - base_ca
            );
        }
    }
}
