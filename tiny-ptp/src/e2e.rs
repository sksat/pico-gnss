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

use crate::message::{Body, Message, PortIdentity};

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
}

/// One complete exchange, as the slave saw it.
#[derive(Clone, Copy, Debug)]
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

/// Turn an exchange into an offset and a path delay.
pub fn measure(exchange: &Exchange) -> Result<Measurement, Reject> {
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
    let t3 = exchange.delay_req_left_ns as i128;
    let t4 = receive.to_ns() as i128;

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
}
