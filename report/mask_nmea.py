# /// script
# requires-python = ">=3.10"
# ///
"""NMEA ログから座標(緯度・経度)をマスクする。

GGA / GLL / RMC の緯度・経度フィールドの数字を 0 で潰し、チェックサムを振り直す
(センテンスは構文的に有効なまま残る)。GST の誤差σ、GSV の az/el、ZDA/VTG、
PPSGEN などタイミング系の行はそのまま保つ。ログを commit する前に通すこと。
defmt ログ(行中に `$G..` センテンスが埋め込まれる)でも生 NMEA でも動く。

Python 環境は uv:
  uv run mask_nmea.py < in.log > out.log
  uv run mask_nmea.py in.log out.log
  uv run mask_nmea.py in.log              # in-place 上書き (in.log.bak にバックアップ)
"""
import re
import sys

# 文型ごとの (緯度, 経度) フィールド位置。"$" を除いたボディを "," で split した 0 始まり index。
#   GGA: $G?GGA,time,LAT,N/S,LON,E/W,...      -> 2, 4
#   GLL: $G?GLL,LAT,N/S,LON,E/W,time,...      -> 1, 3
#   RMC: $G?RMC,time,status,LAT,N/S,LON,E/W,. -> 3, 5
LATLON = {"GGA": (2, 4), "GLL": (1, 3), "RMC": (3, 5)}
# talker(2字) + 文型(3字) で始まる NMEA センテンス全体(任意でチェックサム付き)。
NMEA_RE = re.compile(r"\$G[A-Z]{4}[^*\s]*(?:\*[0-9A-Fa-f]{2})?")


def _checksum(body: str) -> str:
    c = 0
    for ch in body:
        c ^= ord(ch)
    return f"{c:02X}"


def mask_sentence(s: str) -> str:
    star = s.rfind("*")
    body = s[1:star] if star != -1 else s[1:]
    fields = body.split(",")
    typ = fields[0][-3:]  # 末尾3字 = GGA / GLL / RMC / GST ...
    if typ not in LATLON:
        return s
    masked = False
    for i in LATLON[typ]:
        if i < len(fields) and re.search(r"\d", fields[i]):
            fields[i] = re.sub(r"\d", "0", fields[i])  # 桁・小数点の形は保ち、数字を 0 に
            masked = True
    if not masked:
        return s
    nb = ",".join(fields)
    return f"${nb}*{_checksum(nb)}"


def mask_text(text: str) -> str:
    return NMEA_RE.sub(lambda m: mask_sentence(m.group(0)), text)


def main() -> None:
    args = sys.argv[1:]
    if not args:
        sys.stdout.write(mask_text(sys.stdin.read()))
        return
    inp = args[0]
    out = args[1] if len(args) >= 2 else inp
    data = open(inp, encoding="utf-8", errors="replace").read()
    if out == inp:
        open(inp + ".bak", "w", encoding="utf-8").write(data)
    open(out, "w", encoding="utf-8").write(mask_text(data))
    n = len(re.findall(r"\$G[A-Z](?:GGA|GLL|RMC)", data))
    print(f"masked lat/lon in {n} GGA/GLL/RMC sentences: {inp} -> {out}", file=sys.stderr)


if __name__ == "__main__":
    main()
