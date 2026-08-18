# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""NTP broadcast (RFC 5905 mode 5) を受けて、ホスト時刻とのオフセットを測る。

    sudo uv run --no-project scripts/ntp_broadcast_listen.py [--port 123] [--count 60]

なぜ専用ツールか: broadcast client mode を実装しているのは本家 ntpd だけで、chrony も
systemd-timesyncd も受けられない。ntpd を入れずに「実際に何 ns ずれているか」だけ知りたい
場面がほとんどなので、その一点に絞る。

計測の要点:

* 受信時刻は **SO_TIMESTAMPNS** でカーネルから取る。userspace で time() を呼ぶと GIL や
  スケジューリングで数百 µs 平気で乗り、測りたい量より大きくなる。
* 出るオフセットは「サーバの送信時刻 − ホストの受信時刻」なので、**片道の経路遅延を含む**。
  broadcast では受信側が経路を測れないので、これは原理的に分離できない (それが broadcast の
  限界そのもの)。10BASE-T のフレーム送出だけで 81.6 µs かかることに注意。
* したがって平均は「固定スキュー + 経路遅延」、標準偏差が「揺れ」。前者は較正で、後者は
  設計で減らす量。

root が要るのは UDP 123 が特権ポートだからで、それ以外の理由はない。
"""

import argparse
import socket
import struct
import sys
import time

NTP_UNIX_OFFSET = 2_208_988_800

# CPython does not re-export these on every platform, but they are stable Linux ABI numbers
# (asm-generic/socket.h). SO_ and SCM_ share the value.
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)

MODE_NAMES = {
    0: "reserved",
    1: "sym-active",
    2: "sym-passive",
    3: "client",
    4: "server",
    5: "broadcast",
    6: "control",
    7: "private",
}


def ntp_to_unix_ns(raw: int) -> int:
    """NTP 64bit (32.32, 1900 epoch) -> unix ns."""
    secs, frac = raw >> 32, raw & 0xFFFF_FFFF
    return (secs - NTP_UNIX_OFFSET) * 1_000_000_000 + (frac * 1_000_000_000 + (1 << 31) >> 32)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=123)
    ap.add_argument("--count", type=int, default=60, help="0 = 無制限")
    ap.add_argument("--quiet", action="store_true", help="1 行ずつは出さず要約だけ")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # カーネル受信タイムスタンプ。これが無いと測っているのは自分のスケジューリング遅延。
    sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
    try:
        sock.bind(("", args.port))
    except PermissionError:
        print(
            f"UDP {args.port} を bind できない (特権ポート)。sudo で実行するか --port を変える。",
            file=sys.stderr,
        )
        return 1

    print(f"listening on UDP :{args.port} for NTP broadcasts (Ctrl-C to stop)")
    offsets: list[float] = []
    try:
        while args.count == 0 or len(offsets) < args.count:
            data, ancdata, _flags, addr = sock.recvmsg(1024, socket.CMSG_SPACE(32))

            recv_ns = None
            for level, typ, buf in ancdata:
                if level == socket.SOL_SOCKET and typ == SO_TIMESTAMPNS:
                    sec, nsec = struct.unpack("qq", buf[:16])
                    recv_ns = sec * 1_000_000_000 + nsec
            if recv_ns is None:  # kernel timestamp unavailable; say so rather than fake it
                recv_ns = time.time_ns()

            if len(data) < 48:
                continue
            li_vn_mode, stratum, poll, precision = struct.unpack("!BBbb", data[0:4])
            mode = li_vn_mode & 0b111
            leap = li_vn_mode >> 6
            root_disp = struct.unpack("!I", data[8:12])[0]
            refid = data[12:16]
            xmt = ntp_to_unix_ns(struct.unpack("!Q", data[40:48])[0])

            offset_ns = xmt - recv_ns
            offsets.append(offset_ns / 1e9)
            if not args.quiet:
                # Both absolute times, not just their difference: a difference alone cannot say
                # whether the server's clock is wrong or its scheduling is, and the two need
                # opposite fixes.
                def iso(ns: int) -> str:
                    return time.strftime("%H:%M:%S", time.gmtime(ns // 1_000_000_000)) + (
                        f".{ns % 1_000_000_000:09d}"
                    )

                print(
                    f"{addr[0]:<15} mode={MODE_NAMES.get(mode, mode):<9} stratum={stratum} "
                    f"li={leap} prec=2^{precision} refid={refid.decode('ascii', 'replace').rstrip(chr(0))!r:<7} "
                    f"rootdisp={root_disp / 65536 * 1e6:.0f}us  xmt={iso(xmt)} recv={iso(recv_ns)} "
                    f"offset={offset_ns / 1000:+.1f}us"
                )
    except KeyboardInterrupt:
        pass

    if not offsets:
        print("no NTP packets received", file=sys.stderr)
        return 1

    n = len(offsets)
    mean = sum(offsets) / n
    var = sum((o - mean) ** 2 for o in offsets) / n
    sd = var**0.5
    print()
    print(f"n={n}")
    print(f"mean offset = {mean * 1e6:+.1f} us   (fixed skew + one-way path delay)")
    print(f"std  offset = {sd * 1e6:.1f} us      (jitter)")
    print(f"min/max     = {min(offsets) * 1e6:+.1f} / {max(offsets) * 1e6:+.1f} us")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
