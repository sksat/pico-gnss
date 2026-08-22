//! What four timestamps say about two clocks.
//!
//! The end-to-end delay mechanism, IEEE 1588-2008 §11.3. A master sends `Sync` and then says in a
//! `Follow_Up` when the `Sync` actually left; the slave answers with `Delay_Req` and the master
//! says in a `Delay_Resp` when that arrived. Four moments, two of them measured at each end:
//!
//! ```text
//!   master   t1 ──────────────► Sync
//!                               Follow_Up (carries t1)
//!   slave                   t2  arrival
//!   slave    t3 ──────────────► Delay_Req
//!   master                  t4  arrival
//!                               Delay_Resp (carries t4)
//! ```
//!
//! From those: the path delay is half of what the round trip cost after the far side's own residence
//! is taken out, and the offset is how much of the first leg was clock difference rather than path.
//!
//! **The halving is an assumption, not a measurement.** It says the two directions took the same
//! time. Whatever they did not share goes into the offset at half its size, and nothing in the
//! exchange can see it — which is why an asymmetry has to be found by other means, and why this
//! module has a test that says exactly that.

use crate::message::{Body, Message, PortIdentity, Timestamp};

/// Why an exchange was not turned into a measurement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reject {
    /// A message was not the kind that belongs in that slot.
    WrongMessage,
    /// The `Follow_Up` does not name the `Sync`, or the `Delay_Resp` does not name the `Delay_Req`.
    SequenceMismatch,
    /// The `Follow_Up` or the `Delay_Resp` came from a different port than the `Sync`.
    DifferentSource,
    /// The `Delay_Resp` is answering somebody else's request.
    NotForUs,
    /// The `Sync` did not say a `Follow_Up` was coming, so its own timestamp is the one that
    /// counts — and this profile does not read that one, because on this hardware it cannot be
    /// written accurately.
    NotTwoStep,
    /// Nothing is wrong: a `Sync` was taken and the exchange is waiting on its `Follow_Up`.
    AwaitingFollowUp,
}

/// One complete exchange, as the slave saw it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Exchange {
    /// The `Sync`, as received.
    pub sync: Message,
    /// When it arrived, by the slave's clock (t2).
    pub sync_arrived_ns: i64,
    /// The `Follow_Up` that carries the `Sync`'s real departure (t1).
    pub follow_up: Message,
    /// The `Delay_Req` the slave sent.
    pub delay_req: Message,
    /// When it left, by the slave's clock (t3).
    pub delay_req_left_ns: i64,
    /// The `Delay_Resp` that carries its arrival at the master (t4).
    pub delay_resp: Message,
    /// This port, so a response meant for someone else can be told apart from one meant for us.
    pub us: PortIdentity,
}

/// What the exchange came to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Measurement {
    /// How far the slave's clock is ahead of the master's (ns). Subtract it from the slave's clock
    /// to reach the master's — the standard's `offsetFromMaster`, and the opposite sign to the
    /// offset `tiny_ntp` reports.
    pub offset_from_master_ns: i64,
    /// Half the round trip, once the master's residence and any declared corrections are out of it.
    pub mean_path_delay_ns: i64,
    /// The `Sync`'s sequence, so a caller can tell which exchange this was.
    pub sequence: u16,
}

/// Turn an exchange into an offset and a path delay, on the assumption that the slave's counter
/// keeps the master's rate.
///
/// It does not, and the assumption has a price that is easy to miss because it is a constant while
/// the exchange keeps its shape. See [`measure_with_rate`], which is this with the rate given.
pub fn measure(exchange: &Exchange) -> Result<Measurement, Reject> {
    measure_with_rate(exchange, 0)
}

/// The widest rate the correction will believe: ten percent, which no clock this is for is.
const MAX_RATE_PPB: i128 = 100_000_000;

/// The same, told how many parts per billion the slave's counter gains on the master's.
///
/// Between `t2` and `t3` the slave is waiting, and it times that wait on its own counter. A
/// counter that gains reads the wait as longer than it was, and the two-step arithmetic has no way
/// to tell that apart from the wire: half of the stretch comes out of the path delay and half goes
/// into the offset. Two hundred milliseconds of turnaround on a crystal two parts per million out
/// is a couple of hundred nanoseconds of standing error, which is more than everything else the
/// exchange is trying to measure.
///
/// A negative `mean_path_delay_ns` is this, seen: a one-way delay cannot be less than nothing, so
/// a link short enough for the stretch to overtake it reports one.
///
/// The rate is undone over the slave's own wait and nowhere else. `t2` and `t4` are each read once,
/// by whichever clock owns them, and no interval on either side of them belongs to this.
pub fn measure_with_rate(exchange: &Exchange, slave_rate_ppb: i64) -> Result<Measurement, Reject> {
    // Every slot has to hold the kind of message it is named for. A `Follow_Up` in the `Sync` slot
    // would still have a timestamp in it, and the arithmetic would produce a number.
    let Body::Sync(_) = exchange.sync.body else {
        return Err(Reject::WrongMessage);
    };
    let Body::FollowUp(origin) = exchange.follow_up.body else {
        return Err(Reject::WrongMessage);
    };
    let Body::DelayReq(_) = exchange.delay_req.body else {
        return Err(Reject::WrongMessage);
    };
    let Body::DelayResp {
        receive,
        requesting,
    } = exchange.delay_resp.body
    else {
        return Err(Reject::WrongMessage);
    };

    // A `Sync` that promised no `Follow_Up` is carrying its own departure, and this profile does
    // not read that one — on the hardware it was written for, a timestamp inside the message it
    // describes is a claim about a moment that had not happened.
    if !exchange.sync.two_step {
        return Err(Reject::NotTwoStep);
    }
    if exchange.follow_up.sequence != exchange.sync.sequence {
        return Err(Reject::SequenceMismatch);
    }
    if exchange.delay_resp.sequence != exchange.delay_req.sequence {
        return Err(Reject::SequenceMismatch);
    }
    if exchange.follow_up.source != exchange.sync.source
        || exchange.delay_resp.source != exchange.sync.source
    {
        return Err(Reject::DifferentSource);
    }
    if requesting != exchange.us {
        return Err(Reject::NotForUs);
    }

    // Scaled nanoseconds throughout, the unit `correctionField` is in, so nothing is rounded until
    // the end. The differences are microseconds to milliseconds even though the timestamps
    // themselves are eighteen digits, which is why shifting them is safe.
    const SCALE: i128 = 65_536;
    let t1 = origin.to_ns() as i128;
    let t2 = exchange.sync_arrived_ns as i128;
    let t4 = receive.to_ns() as i128;
    // The slave's wait, as the master's counter would have read it.
    //
    // The rate is bounded before it divides. A crystal is parts per million out and an estimate
    // that says otherwise is a broken estimate, not a fast crystal — and at minus one part per one
    // the divisor is zero, which is a panic on a board with nobody to catch it.
    let rate = (slave_rate_ppb as i128).clamp(-MAX_RATE_PPB, MAX_RATE_PPB);
    let waited = exchange.delay_req_left_ns as i128 - exchange.sync_arrived_ns as i128;
    let t3 = t2 + waited * 1_000_000_000 / (1_000_000_000 + rate);

    // The master's residence and any transparent clock's, as declared on each leg.
    let to_slave = (t2 - t1) * SCALE
        - exchange.sync.correction_sub_ns as i128
        - exchange.follow_up.correction_sub_ns as i128;
    let to_master = (t4 - t3) * SCALE - exchange.delay_resp.correction_sub_ns as i128;

    // Half of the round trip is the path — an assumption about symmetry, and the only one here.
    let mean_path_delay = (to_slave + to_master) / 2;
    // What is left of the first leg once the path is out of it is clock difference.
    let offset = to_slave - mean_path_delay;

    Ok(Measurement {
        offset_from_master_ns: (offset / SCALE) as i64,
        mean_path_delay_ns: (mean_path_delay / SCALE) as i64,
        sequence: exchange.sync.sequence,
    })
}

/// What a slave should do with the message it just took off the wire.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    /// Nothing, and why.
    Ignored(Reject),
    /// The master's half is complete. Send this, then say when it left.
    SendDelayReq(Message),
    /// All four moments are in. Hand it to [`measure`].
    Complete(Exchange),
}

/// The slave half of the exchange, as a state machine over arriving messages.
///
/// It lives here rather than in the firmware because every rule it keeps is a rule about message
/// ordering, and none of them need hardware to exercise. A `Follow_Up` that crosses the next
/// `Sync`, a `Delay_Resp` answering a request two sequences old, a response addressed to the other
/// port: each is a way to compute an offset out of timestamps that do not belong together, and each
/// is cheaper to refuse than to explain afterwards.
#[derive(Clone, Copy, Debug)]
pub struct Slave {
    us: PortIdentity,
    /// A `Sync` waiting for the `Follow_Up` that says when it really left.
    pending: Option<(Message, i64)>,
    /// A `Delay_Req` we have handed out: the request, when it left (once known), and the master's
    /// half of the exchange it belongs to.
    outstanding: Option<Outstanding>,
    next_sequence: u16,
}

#[derive(Clone, Copy, Debug)]
struct Outstanding {
    delay_req: Message,
    left_ns: Option<i64>,
    sync: Message,
    sync_arrived_ns: i64,
    follow_up: Message,
}

impl Slave {
    pub const fn new(us: PortIdentity) -> Self {
        Self {
            us,
            pending: None,
            outstanding: None,
            next_sequence: 0,
        }
    }

    /// This port's identity, as it appears in the `requestingPortIdentity` of a `Delay_Resp`.
    pub const fn port(&self) -> PortIdentity {
        self.us
    }

    /// A message came off the wire at `arrived_ns` by this clock.
    pub fn on_message(&mut self, msg: Message, arrived_ns: i64) -> Action {
        match msg.body {
            Body::Sync(_) => {
                // A Sync arriving while one is pending means that one's Follow_Up is not coming.
                // Keeping the older one would let a late Follow_Up be paired with the wrong moment
                // on the wire, so the newer Sync replaces it outright.
                self.pending = Some((msg, arrived_ns));
                Action::Ignored(Reject::AwaitingFollowUp)
            }
            Body::FollowUp(_) => self.on_follow_up(msg),
            Body::DelayResp { requesting, .. } => self.on_delay_resp(msg, requesting),
            Body::DelayReq(_) => Action::Ignored(Reject::WrongMessage),
        }
    }

    fn on_follow_up(&mut self, follow_up: Message) -> Action {
        let Some((sync, sync_arrived_ns)) = self.pending else {
            return Action::Ignored(Reject::SequenceMismatch);
        };
        if follow_up.sequence != sync.sequence {
            return Action::Ignored(Reject::SequenceMismatch);
        }
        if follow_up.source != sync.source {
            return Action::Ignored(Reject::DifferentSource);
        }
        self.pending = None;

        let delay_req = Message {
            domain: sync.domain,
            two_step: false,
            correction_sub_ns: 0,
            source: self.us,
            sequence: self.next_sequence,
            // A Delay_Req is sent when the slave decides to, so it has no interval to declare.
            log_interval: 0x7F_u8 as i8,
            body: Body::DelayReq(Timestamp::default()),
        };
        self.next_sequence = self.next_sequence.wrapping_add(1);
        self.outstanding = Some(Outstanding {
            delay_req,
            left_ns: None,
            sync,
            sync_arrived_ns,
            follow_up,
        });
        Action::SendDelayReq(delay_req)
    }

    fn on_delay_resp(&mut self, delay_resp: Message, requesting: PortIdentity) -> Action {
        let Some(out) = self.outstanding else {
            return Action::Ignored(Reject::SequenceMismatch);
        };
        if requesting != self.us {
            return Action::Ignored(Reject::NotForUs);
        }
        if delay_resp.sequence != out.delay_req.sequence {
            return Action::Ignored(Reject::SequenceMismatch);
        }
        let Some(delay_req_left_ns) = out.left_ns else {
            return Action::Ignored(Reject::WrongMessage);
        };
        // One response per request. A duplicate would produce a second measurement from the same
        // four moments, which reads as new information and is not.
        self.outstanding = None;
        Action::Complete(Exchange {
            sync: out.sync,
            sync_arrived_ns: out.sync_arrived_ns,
            follow_up: out.follow_up,
            delay_req: out.delay_req,
            delay_req_left_ns,
            delay_resp,
            us: self.us,
        })
    }

    /// The `Delay_Req` handed out by [`Action::SendDelayReq`] left the pin at `left_ns`.
    ///
    /// Until this is called the exchange cannot complete: t3 is the one moment the slave measures
    /// about itself, and a response arriving before it is refused rather than guessed at.
    pub fn on_delay_req_sent(&mut self, left_ns: i64) {
        if let Some(out) = self.outstanding.as_mut() {
            out.left_ns = Some(left_ns);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::{Body, ClockIdentity, Message, PortIdentity, Timestamp};

    fn master() -> PortIdentity {
        PortIdentity {
            clock: ClockIdentity([1, 1, 1, 1, 1, 1, 1, 1]),
            port: 1,
        }
    }

    fn slave() -> PortIdentity {
        PortIdentity {
            clock: ClockIdentity([2, 2, 2, 2, 2, 2, 2, 2]),
            port: 1,
        }
    }

    fn msg(source: PortIdentity, sequence: u16, body: Body) -> Message {
        Message {
            domain: 0,
            two_step: matches!(body, Body::Sync(_)),
            correction_sub_ns: 0,
            source,
            sequence,
            log_interval: 0,
            body,
        }
    }

    /// An exchange over a path that took `to_slave` and `back` nanoseconds, with the slave's clock
    /// `offset` nanoseconds ahead of the master's.
    fn exchange(offset: i64, to_slave: i64, back: i64) -> Exchange {
        // Master time of the Sync leaving, and of the Delay_Req arriving.
        let t1 = 1_787_180_000_000_000_000i64;
        // Slave time of the Sync arriving: master time plus the path, plus the clock difference.
        let t2 = t1 + to_slave + offset;
        // The slave waits a while before asking back.
        let t3 = t2 + 10_000_000;
        let t4 = t3 - offset + back;

        Exchange {
            sync: msg(master(), 7, Body::Sync(Timestamp::default())),
            sync_arrived_ns: t2,
            follow_up: msg(master(), 7, Body::FollowUp(Timestamp::from_ns(t1))),
            delay_req: msg(slave(), 9, Body::DelayReq(Timestamp::default())),
            delay_req_left_ns: t3,
            delay_resp: msg(
                master(),
                9,
                Body::DelayResp {
                    receive: Timestamp::from_ns(t4),
                    requesting: slave(),
                },
            ),
            us: slave(),
        }
    }

    #[test]
    fn a_symmetric_path_gives_back_the_offset_and_the_delay() {
        let m = measure(&exchange(1_500, 20_000, 20_000)).expect("measures");
        assert_eq!(m.offset_from_master_ns, 1_500);
        assert_eq!(m.mean_path_delay_ns, 20_000);
        assert_eq!(m.sequence, 7);
    }

    #[test]
    fn a_clock_that_agrees_reports_no_offset() {
        let m = measure(&exchange(0, 12_345, 12_345)).expect("measures");
        assert_eq!(m.offset_from_master_ns, 0);
        assert_eq!(m.mean_path_delay_ns, 12_345);
    }

    #[test]
    fn an_asymmetric_path_puts_half_of_the_difference_into_the_offset() {
        // The two directions differ by 8 us, and the clocks agree exactly. There is nothing in the
        // exchange that can tell those apart, so half the asymmetry is reported as offset. This is
        // the mechanism's central limitation and not a defect in the arithmetic.
        let m = measure(&exchange(0, 24_000, 16_000)).expect("measures");
        assert_eq!(m.offset_from_master_ns, 4_000);
        assert_eq!(m.mean_path_delay_ns, 20_000);
    }

    /// An exchange as a slave whose counter gains `rate_ppb` on the master's would time it.
    ///
    /// The two scales are set to agree, but for `offset`, at the moment the `Sync` leaves; from
    /// there the slave's own reading of any elapsed time is stretched by the rate. `path` is the
    /// one-way delay and `turnaround` is how long the slave holds before asking back, both in the
    /// master's time.
    fn drifting_exchange(offset: i64, path: i64, turnaround: i64, rate_ppb: i64) -> Exchange {
        let t1 = 1_787_180_000_000_000_000i64;
        let slave_reads = |elapsed: i64| t1 + offset + elapsed + elapsed * rate_ppb / 1_000_000_000;
        let mut e = exchange(0, 0, 0);
        e.sync_arrived_ns = slave_reads(path);
        e.delay_req_left_ns = slave_reads(path + turnaround);
        e.follow_up = msg(master(), 7, Body::FollowUp(Timestamp::from_ns(t1)));
        e.delay_resp = msg(
            master(),
            9,
            Body::DelayResp {
                receive: Timestamp::from_ns(t1 + turnaround + 2 * path),
                requesting: slave(),
            },
        );
        e
    }

    #[test]
    fn a_slave_that_gains_leaves_half_its_turnaround_in_the_offset() {
        // Two hundred milliseconds of turnaround on a counter 1830 parts per billion fast is 366 ns
        // of stretch, and the arithmetic splits it: half comes out of the path and half goes into
        // the offset. The path is what shows it - a one-way delay cannot be shorter than it is, and
        // with a jumper for a path it goes negative.
        let e = drifting_exchange(0, 1_000, 200_000_000, 1_830);
        let m = measure(&e).expect("measures");
        assert_eq!(m.mean_path_delay_ns, 1_000 - 183, "half out of the path");
        assert_eq!(m.offset_from_master_ns, 183, "and half into the offset");

        // Told what the slave's counter does, the same four moments give back what they were built
        // from. Nothing else changes: the rate is undone over the slave's own wait and nowhere else.
        let m = measure_with_rate(&e, 1_830).expect("measures");
        assert_eq!(m.mean_path_delay_ns, 1_000);
        assert_eq!(m.offset_from_master_ns, 0);
    }

    #[test]
    fn an_impossible_rate_is_bounded_rather_than_believed() {
        // Minus one part per one would put a zero under the division. Nothing is expected to reach
        // this; it is here so that a discipline that has come apart cannot take the board with it.
        let e = drifting_exchange(0, 1_000, 200_000_000, 1_830);
        for rate in [-1_000_000_000, i64::MIN, i64::MAX, 999_999_999_999] {
            let m = measure_with_rate(&e, rate).expect("measures");
            assert_eq!(
                m,
                measure_with_rate(&e, rate.signum() * 100_000_000).expect("measures"),
                "rate {rate} is the bound"
            );
        }
    }

    #[test]
    fn the_bias_is_proportional_to_the_turnaround() {
        // Which is what tells it apart from a fixed skew, on the board as well as here.
        for (turnaround, expected) in [(50_000_000, 45), (200_000_000, 183), (500_000_000, 457)] {
            let m = measure(&drifting_exchange(0, 1_000, turnaround, 1_830)).expect("measures");
            assert_eq!(m.offset_from_master_ns, expected, "turnaround {turnaround}");
        }
    }

    #[test]
    fn a_correction_is_taken_out_in_the_units_the_standard_uses() {
        // correctionField is nanoseconds scaled by 2^16. Three microseconds declared on the first
        // leg is three microseconds of path that was not clock difference.
        let mut e = exchange(0, 20_000, 20_000);
        e.follow_up.correction_sub_ns = 3_000 * 65_536;
        let m = measure(&e).expect("measures");
        assert_eq!(m.offset_from_master_ns, -1_500);
        assert_eq!(m.mean_path_delay_ns, 18_500);
    }

    #[test]
    fn corrections_on_both_legs_cancel_out_of_the_offset() {
        let mut e = exchange(2_000, 20_000, 20_000);
        e.sync.correction_sub_ns = 1_000 * 65_536;
        e.delay_resp.correction_sub_ns = 1_000 * 65_536;
        let m = measure(&e).expect("measures");
        assert_eq!(m.offset_from_master_ns, 2_000);
        assert_eq!(m.mean_path_delay_ns, 19_000);
    }

    #[test]
    fn a_follow_up_that_names_another_sync_is_refused() {
        let mut e = exchange(0, 20_000, 20_000);
        e.follow_up.sequence = 8;
        assert_eq!(measure(&e), Err(Reject::SequenceMismatch));
    }

    #[test]
    fn a_response_that_names_another_request_is_refused() {
        let mut e = exchange(0, 20_000, 20_000);
        e.delay_resp.sequence = 10;
        assert_eq!(measure(&e), Err(Reject::SequenceMismatch));
    }

    #[test]
    fn a_response_meant_for_another_port_is_refused() {
        let mut e = exchange(0, 20_000, 20_000);
        e.delay_resp.body = Body::DelayResp {
            receive: Timestamp::from_ns(1_787_180_000_000_000_000),
            requesting: PortIdentity {
                clock: ClockIdentity([9, 9, 9, 9, 9, 9, 9, 9]),
                port: 1,
            },
        };
        assert_eq!(measure(&e), Err(Reject::NotForUs));
    }

    #[test]
    fn a_follow_up_from_a_different_clock_is_refused() {
        let mut e = exchange(0, 20_000, 20_000);
        e.follow_up.source = slave();
        assert_eq!(measure(&e), Err(Reject::DifferentSource));
    }

    #[test]
    fn a_one_step_sync_is_refused() {
        let mut e = exchange(0, 20_000, 20_000);
        e.sync.two_step = false;
        assert_eq!(measure(&e), Err(Reject::NotTwoStep));
    }

    #[test]
    fn a_message_in_the_wrong_slot_is_refused() {
        let mut e = exchange(0, 20_000, 20_000);
        e.follow_up.body = Body::Sync(Timestamp::default());
        assert_eq!(measure(&e), Err(Reject::WrongMessage));

        let mut e = exchange(0, 20_000, 20_000);
        e.sync.body = Body::FollowUp(Timestamp::default());
        assert_eq!(measure(&e), Err(Reject::WrongMessage));

        let mut e = exchange(0, 20_000, 20_000);
        e.delay_resp.body = Body::DelayReq(Timestamp::default());
        assert_eq!(measure(&e), Err(Reject::WrongMessage));
    }

    // --- the slave's state machine ---

    fn sync(sequence: u16) -> Message {
        msg(master(), sequence, Body::Sync(Timestamp::default()))
    }

    fn follow_up(sequence: u16, t1: i64) -> Message {
        msg(master(), sequence, Body::FollowUp(Timestamp::from_ns(t1)))
    }

    fn delay_resp(sequence: u16, t4: i64, requesting: PortIdentity) -> Message {
        msg(
            master(),
            sequence,
            Body::DelayResp {
                receive: Timestamp::from_ns(t4),
                requesting,
            },
        )
    }

    /// Drive a slave up to the point where it has handed out a `Delay_Req`.
    fn armed(t1: i64, t2: i64) -> (Slave, Message) {
        let mut s = Slave::new(slave());
        assert_eq!(
            s.on_message(sync(7), t2),
            Action::Ignored(Reject::AwaitingFollowUp)
        );
        match s.on_message(follow_up(7, t1), t2 + 1) {
            Action::SendDelayReq(req) => (s, req),
            other => panic!("expected a Delay_Req to go out, got {other:?}"),
        }
    }

    #[test]
    fn a_sync_alone_does_not_start_a_request() {
        // The Sync's own timestamp is a placeholder in two-step, so there is nothing to answer
        // until the Follow_Up says when it really left.
        let mut s = Slave::new(slave());
        assert!(matches!(s.on_message(sync(1), 1_000), Action::Ignored(_)));
    }

    #[test]
    fn a_follow_up_completes_the_masters_half_and_asks_for_the_return_leg() {
        let (_, req) = armed(1_787_180_000_000_000_000, 1_787_180_000_000_020_000);
        assert!(matches!(req.body, Body::DelayReq(_)));
        assert_eq!(req.source, slave());
    }

    #[test]
    fn a_follow_up_for_another_sequence_is_refused() {
        let mut s = Slave::new(slave());
        s.on_message(sync(7), 1_000);
        assert_eq!(
            s.on_message(follow_up(8, 900), 1_100),
            Action::Ignored(Reject::SequenceMismatch)
        );
    }

    #[test]
    fn a_follow_up_from_another_port_is_refused() {
        let mut s = Slave::new(slave());
        s.on_message(sync(7), 1_000);
        let stranger = msg(slave(), 7, Body::FollowUp(Timestamp::from_ns(900)));
        assert_eq!(
            s.on_message(stranger, 1_100),
            Action::Ignored(Reject::DifferentSource)
        );
    }

    #[test]
    fn a_sync_that_crosses_its_follow_up_replaces_the_pending_one() {
        // The Follow_Up for 7 was lost. Its timestamp arriving late must not be paired with the
        // Sync for 8, which is a different moment on the wire.
        let mut s = Slave::new(slave());
        s.on_message(sync(7), 1_000);
        s.on_message(sync(8), 2_000);
        assert_eq!(
            s.on_message(follow_up(7, 900), 2_100),
            Action::Ignored(Reject::SequenceMismatch)
        );
        assert!(matches!(
            s.on_message(follow_up(8, 1_900), 2_200),
            Action::SendDelayReq(_)
        ));
    }

    #[test]
    fn a_response_before_the_request_left_is_refused() {
        // t3 is the one moment the slave measures about itself. Without it there are three
        // timestamps, not four.
        let (mut s, req) = armed(1_787_180_000_000_000_000, 1_787_180_000_000_020_000);
        let resp = delay_resp(req.sequence, 1_787_180_000_000_060_000, slave());
        assert!(matches!(s.on_message(resp, 0), Action::Ignored(_)));
    }

    #[test]
    fn a_response_addressed_to_the_other_port_is_refused() {
        let (mut s, req) = armed(1_787_180_000_000_000_000, 1_787_180_000_000_020_000);
        s.on_delay_req_sent(1_787_180_000_000_040_000);
        let resp = delay_resp(req.sequence, 1_787_180_000_000_060_000, master());
        assert_eq!(s.on_message(resp, 0), Action::Ignored(Reject::NotForUs));
    }

    #[test]
    fn a_response_to_a_request_two_sequences_old_is_refused() {
        let (mut s, req) = armed(1_787_180_000_000_000_000, 1_787_180_000_000_020_000);
        s.on_delay_req_sent(1_787_180_000_000_040_000);
        let stale = delay_resp(
            req.sequence.wrapping_sub(2),
            1_787_180_000_000_060_000,
            slave(),
        );
        assert_eq!(
            s.on_message(stale, 0),
            Action::Ignored(Reject::SequenceMismatch)
        );
    }

    #[test]
    fn a_complete_exchange_measures_the_offset_the_four_moments_imply() {
        // Path 20 µs each way, slave 5 µs ahead.
        let t1 = 1_787_180_000_000_000_000i64;
        let (mut s, req) = armed(t1, t1 + 20_000 + 5_000);
        let t3 = t1 + 40_000 + 5_000;
        s.on_delay_req_sent(t3);
        let resp = delay_resp(req.sequence, t3 - 5_000 + 20_000, slave());
        let Action::Complete(done) = s.on_message(resp, 0) else {
            panic!("expected the exchange to complete");
        };
        let m = measure(&done).expect("four timestamps that belong together");
        assert_eq!(m.offset_from_master_ns, 5_000);
        assert_eq!(m.mean_path_delay_ns, 20_000);
    }

    #[test]
    fn a_second_response_to_the_same_request_is_refused() {
        let t1 = 1_787_180_000_000_000_000i64;
        let (mut s, req) = armed(t1, t1 + 25_000);
        s.on_delay_req_sent(t1 + 45_000);
        let resp = delay_resp(req.sequence, t1 + 65_000, slave());
        assert!(matches!(s.on_message(resp, 0), Action::Complete(_)));
        assert!(matches!(s.on_message(resp, 0), Action::Ignored(_)));
    }

    #[test]
    fn successive_exchanges_use_successive_sequences() {
        let t1 = 1_787_180_000_000_000_000i64;
        let (mut s, first) = armed(t1, t1 + 25_000);
        s.on_delay_req_sent(t1 + 45_000);
        s.on_message(delay_resp(first.sequence, t1 + 65_000, slave()), 0);

        s.on_message(sync(9), t1 + 1_000_025_000);
        let Action::SendDelayReq(second) = s.on_message(follow_up(9, t1 + 1_000_000_000), 0) else {
            panic!("expected a second request");
        };
        assert_ne!(second.sequence, first.sequence);
    }
}
