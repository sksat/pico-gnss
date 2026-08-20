# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""受信側ホストの時計が、外の基準からどれだけ離れているかを測る。

    uv run docs/report/ntp-stratum1/logs/paths/check_host_clock.py [--server ntp.nict.jp] [--count 20]

このレポートの `−3816 µs` は、動かないスキューと片道の経路遅延とホスト時計の誤差の合計である。
3 つのうちホスト時計だけは、外の基準と往復すれば分けられる。unicast の往復なので RFC 5905 §8 の
式がそのまま使え、経路の遅延は補正して落ちる。

送信元ポートは ephemeral でよく、特権は要らない。宛先の 123 は外向きなので bind しない。

出るのは「このホストの時計 − 基準の時計」で、正なら進んでいる。
最小 delay のサンプルを併記する。往復が短いほど非対称の余地が小さく、その 1 つが最も信用できる。
"""

import argparse
import socket
import statistics
import struct
import sys
import time

NTP_UNIX_OFFSET = 2_208_988_800


def to_unix(raw: int) -> float:
    return raw / 2**32 - NTP_UNIX_OFFSET


def exchange(sock: socket.socket, addr) -> tuple[float, float] | None:
    """1 往復して (offset, delay) を秒で返す。落ちたら None。"""
    packet = bytearray(48)
    packet[0] = 0b00_100_011  # LI=0, VN=4, mode=3 (client)
    t1 = time.time()
    struct.pack_into("!Q", packet, 40, int((t1 + NTP_UNIX_OFFSET) * 2**32))
    try:
        sock.sendto(bytes(packet), addr)
        data, _ = sock.recvfrom(1024)
    except (TimeoutError, OSError):
        return None
    t4 = time.time()
    if len(data) < 48:
        return None

    t2 = to_unix(struct.unpack("!Q", data[32:40])[0])
    t3 = to_unix(struct.unpack("!Q", data[40:48])[0])
    # 送った時刻は自分で覚えている t1 を使う。server が echo する origin と一致するはずで、
    # 一致しなければ別の応答なので捨てる。
    if abs(to_unix(struct.unpack("!Q", data[24:32])[0]) - t1) > 1e-6:
        return None
    return ((t2 - t1) + (t3 - t4)) / 2, (t4 - t1) - (t3 - t2)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="ntp.nict.jp")
    ap.add_argument("--count", type=int, default=20)
    # 既定経路が無線だと往復が数十 ms 揺れて、min-delay を選んでも残差が大きい。
    # 有線側のアドレスに縛ると、同じ L2 なら有線から出ていく。
    ap.add_argument("--bind", help="送信元アドレス (例: 有線側の IP)")
    args = ap.parse_args()

    addr = (socket.gethostbyname(args.server), 123)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    if args.bind:
        sock.bind((args.bind, 0))

    samples = []
    for _ in range(args.count):
        got = exchange(sock, addr)
        if got:
            samples.append(got)
        time.sleep(0.2)

    if not samples:
        print(f"no reply from {args.server}", file=sys.stderr)
        return 1

    offsets = [o for o, _ in samples]
    delays = [d for _, d in samples]
    best = min(samples, key=lambda s: s[1])
    print(f"{args.server}: n={len(samples)}")
    print(f"  offset  mean {statistics.mean(offsets) * 1e6:+.1f} us   median {statistics.median(offsets) * 1e6:+.1f} us")
    print(f"          sd   {statistics.pstdev(offsets) * 1e6:.1f} us")
    print(f"  delay   min  {min(delays) * 1e3:.1f} ms   median {statistics.median(delays) * 1e3:.1f} ms")
    print(f"  最小 delay のサンプル: offset {best[0] * 1e6:+.1f} us (delay {best[1] * 1e3:.1f} ms)")
    print("  正なら、このホストの時計は基準より進んでいる。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
