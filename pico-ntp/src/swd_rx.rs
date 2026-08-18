//! A receive path over the debug probe, so a real NTP client can hold a unicast exchange with this
//! server.
//!
//! # Why
//!
//! The wiring transmits and nothing else, so [`broadcast`](tiny_ntp::server::broadcast) is all it
//! can do on the wire — and a broadcast client cannot measure the path, only assume it. Worse, the
//! clients most people run (chrony, systemd-timesyncd) do not implement broadcast client mode at
//! all. So the question "does a real client accept this time, and how does it converge" cannot be
//! asked over the link as built.
//!
//! SWD can carry it. The probe reads and writes target RAM while the core runs, so a mailbox here
//! plus a bridge on the host is enough to put a genuine mode 3 → mode 4 exchange in front of
//! [`respond`](tiny_ntp::server::respond), with this firmware's own disciplined clock on both
//! timestamps.
//!
//! # What it is not
//!
//! **Not a service, and not a measurement of the link.** The request arrives over USB and the debug
//! probe, so the round trip a client measures describes that path, not Ethernet. What it does
//! measure is everything above the wire: the packet, the epoch, the stratum, and whether a real
//! implementation's loop settles on our time.
//!
//! Gated behind the `swd-rx` feature and absent from a normal build.
//!
//! # Protocol
//!
//! One mailbox, polled. The host writes [`Mailbox::request`] and then increments
//! [`Mailbox::doorbell`]; this task notices `doorbell != done`, timestamps, answers, and sets
//! `done = doorbell` last of all. The host waits for that and reads the reply.
//!
//! Ordering is by the single-writer discipline rather than by barriers: each field has exactly one
//! writer, and the two counters are written after the payloads they describe. The core is a
//! cacheless M0+ and the probe writes the same physical RAM, so there is nothing between the two
//! views to be stale.

use core::ptr::{addr_of, addr_of_mut};

use defmt::{info, warn};
use embassy_time::Timer;

use tiny_ntp::packet::{NtpPacket, PACKET_LEN};
use tiny_ntp::server::{ServeDecision, respond};

/// How often to look at the doorbell. A client polls every few seconds at the fastest, so this only
/// has to be short against the round trip the host is willing to see.
const POLL_MS: u64 = 2;

/// The RAM handshake. `#[repr(C)]` because the host reads it by offset from the symbol address.
#[repr(C)]
pub struct Mailbox {
    /// Written by the host, after `request`. Firmware acts when it differs from `done`.
    pub doorbell: u32,
    /// Written by firmware, after `reply` and the timestamps. Host waits for it to match.
    pub done: u32,
    /// T2: when the firmware saw the request, by the disciplined clock (Unix ns).
    pub recv_unix_ns: i64,
    /// T3: when the firmware finished the reply, likewise.
    pub xmit_unix_ns: i64,
    /// Why no reply was produced, as `SilentReason as u32 + 1`; 0 when one was.
    pub silent: u32,
    pub request: [u8; PACKET_LEN],
    pub reply: [u8; PACKET_LEN],
}

/// The mailbox itself, found by the host through the ELF symbol table.
///
/// `no_mangle` so the name survives to be looked up, and `static mut` because the probe writes into
/// it from outside the program — the compiler must not assume it knows the contents.
#[unsafe(no_mangle)]
pub static mut NTP_SWD_MAILBOX: Mailbox = Mailbox {
    doorbell: 0,
    done: 0,
    recv_unix_ns: 0,
    xmit_unix_ns: 0,
    silent: 0,
    request: [0; PACKET_LEN],
    reply: [0; PACKET_LEN],
};

/// Poll the mailbox and answer whatever the host puts in it.
#[embassy_executor::task]
pub async fn swd_rx_task() {
    info!("SWD RX: mailbox at {}", addr_of!(NTP_SWD_MAILBOX) as u32);
    let mut served: u32 = 0;
    loop {
        Timer::after_millis(POLL_MS).await;

        // Volatile throughout: the writer is on the other side of the debug port, so nothing the
        // compiler can see accounts for these changing.
        let (doorbell, done) = unsafe {
            (
                addr_of!(NTP_SWD_MAILBOX.doorbell).read_volatile(),
                addr_of!(NTP_SWD_MAILBOX.done).read_volatile(),
            )
        };
        if doorbell == done {
            continue;
        }

        // T2 first, before any work: it is when the request was seen.
        let (_, state) = crate::clock_state();
        let recv_unix_ns = crate::clock_state().0.unwrap_or(0);
        let mut buf = [0u8; PACKET_LEN];
        for (i, b) in buf.iter_mut().enumerate() {
            *b = unsafe { addr_of!(NTP_SWD_MAILBOX.request[i]).read_volatile() };
        }

        let (reply, silent) = match NtpPacket::decode(&buf) {
            None => {
                warn!("SWD RX: undecodable request");
                (None, u32::MAX)
            }
            Some(request) => {
                let xmit = crate::clock_state().0.unwrap_or(recv_unix_ns);
                match respond(&crate::CFG, &state, &request, recv_unix_ns, xmit) {
                    ServeDecision::Serve(p) => (Some((p, xmit)), 0),
                    ServeDecision::Silent(reason) => {
                        warn!("SWD RX: silent — {}", defmt::Debug2Format(&reason));
                        (None, reason as u32 + 1)
                    }
                }
            }
        };

        let xmit_unix_ns = match reply {
            Some((packet, xmit)) => {
                let bytes = packet.encode();
                for (i, b) in bytes.iter().enumerate() {
                    unsafe { addr_of_mut!(NTP_SWD_MAILBOX.reply[i]).write_volatile(*b) };
                }
                xmit
            }
            None => recv_unix_ns,
        };

        unsafe {
            addr_of_mut!(NTP_SWD_MAILBOX.recv_unix_ns).write_volatile(recv_unix_ns);
            addr_of_mut!(NTP_SWD_MAILBOX.xmit_unix_ns).write_volatile(xmit_unix_ns);
            addr_of_mut!(NTP_SWD_MAILBOX.silent).write_volatile(silent);
            // Last: this is what the host is waiting on, so everything it describes is in place.
            addr_of_mut!(NTP_SWD_MAILBOX.done).write_volatile(doorbell);
        }

        served = served.wrapping_add(1);
        if served <= 3 {
            info!("SWD RX: answered {} (silent={})", served, silent);
        }
    }
}
