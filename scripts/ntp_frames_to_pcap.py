# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""単発 (この試行専用): pico-ntp の RTT ログから NTPFRAME 行を拾って pcap にする。

    uv run frames_to_pcap.py rtt-ntp2.log frames.pcap

実機が実際に組んだ Ethernet フレームを Wireshark に判定させるための橋渡し。
有線 NIC もオシロも無い状況で、PHY の手前までを実出力で検証する唯一の経路。
バイト列は firmware の defmt 出力から機械的に取り出す (手で書き写さない)。
"""

import re
import subprocess
import sys
from pathlib import Path

FRAME_RE = re.compile(r"NTPFRAME n=(\d+) bytes=\[([0-9a-f, ]+)\]")


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    log, pcap = Path(sys.argv[1]), Path(sys.argv[2])

    frames = []
    for line in log.read_text(errors="replace").splitlines():
        m = FRAME_RE.search(line)
        if m:
            frames.append([int(b, 16) for b in m.group(2).replace(",", " ").split()])
    if not frames:
        print(f"no NTPFRAME lines in {log}", file=sys.stderr)
        return 1

    # text2pcap wants an offset column then hex octets; a blank line separates packets.
    hexdump = []
    for f in frames:
        for off in range(0, len(f), 16):
            chunk = f[off : off + 16]
            hexdump.append(f"{off:06x} " + " ".join(f"{b:02x}" for b in chunk))
        hexdump.append("")
    tmp = pcap.with_suffix(".hex")
    tmp.write_text("\n".join(hexdump) + "\n")

    subprocess.run(["text2pcap", str(tmp), str(pcap)], check=True, capture_output=True)
    print(
        f"{len(frames)} frame(s) -> {pcap}  ({', '.join(str(len(f)) for f in frames)} bytes)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
