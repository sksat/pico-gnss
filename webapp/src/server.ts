/**
 * pico-gnss real-time dashboard bridge.
 *
 * probe-rs の RTT (defmt) 出力、または replay ファイルから firmware の行
 *   - `NMEA $GxXXX,...*hh`
 *   - `PPS count=<n> interval_us=<us> state=<...> missed=<m>`
 *   - `SYNC pps_local_us=<t> unix_s=<s> drift_us=<d>`
 * を抽出し、WebSocket でブラウザへ配信する。静的ファイルも同一ポートで配る。
 *
 * 実行:
 *   node dist/server.js                 # probe-rs run でフラッシュ+配信 (実機)
 *   node dist/server.js --attach        # 既にフラッシュ済みの実機に attach
 *   node dist/server.js --replay sample.log   # 録画 replay (ハード不要)
 * オプション: --port <n> --chip <chip> --elf <path>
 *
 * ランタイム依存ゼロ (Node 組み込みのみ)。WebSocket はサーバ→クライアントの
 * テキストフレーム送出だけを最小実装。
 */
import * as http from "http";
import * as fs from "fs";
import * as path from "path";
import * as readline from "readline";
import { spawn, ChildProcessWithoutNullStreams } from "child_process";
import { createHash } from "crypto";
import { Socket } from "net";
import { setTimeout as sleep } from "timers/promises";

// --- firmware が出す行 → ブラウザへ送る JSON メッセージ ---
type Msg =
  | { t: "nmea"; s: string }
  | { t: "pps"; count: number; interval_us: number; interval_ns: number; state: string; missed: number }
  | { t: "sync"; pps_local_us: number; unix_s: number; drift_us: number; err_ns: number }
  | { t: "time"; unix_ns: number; ppb: number; holdover_ms: number; locked: boolean }
  | { t: "ppsout"; unix_s: number; late_us: number; holdover_ms: number }
  | { t: "fw"; s: string }
  | { t: "status"; source: string; connected: boolean; note?: string };

const NMEA_RE = /NMEA (\$[A-Za-z0-9]{2,5},[^*\s]*\*[0-9A-Fa-f]{2})/;
const PPS_RE = /PPS count=(\d+) interval_us=(\d+) interval_ns=(\d+) state=(\w+) missed=(\d+)/;
const SYNC_RE = /SYNC pps_local_us=(\d+) unix_s=(\d+) drift_us=(-?\d+)(?: err_ns=(-?\d+))?/;
const TIME_RE = /TIME unix_ns=(\d+) ppb=(-?\d+) holdover_ms=(\d+) locked=([01])/;
const PPSOUT_RE = /PPSOUT unix_s=(\d+) sched_us=\d+ fired_us=\d+ late_us=(-?\d+) holdover_ms=(\d+)/;
const FW_RE = /FW (\$PMTK705,[^*]*\*[0-9A-Fa-f]{2})/;

function parseLine(line: string): Msg | null {
  let m: RegExpExecArray | null;
  if ((m = FW_RE.exec(line))) return { t: "fw", s: m[1] };
  if ((m = NMEA_RE.exec(line))) return { t: "nmea", s: m[1] };
  if ((m = PPS_RE.exec(line)))
    return { t: "pps", count: +m[1], interval_us: +m[2], interval_ns: +m[3], state: m[4], missed: +m[5] };
  if ((m = SYNC_RE.exec(line)))
    return { t: "sync", pps_local_us: +m[1], unix_s: +m[2], drift_us: +m[3], err_ns: m[4] ? +m[4] : 0 };
  if ((m = TIME_RE.exec(line)))
    return { t: "time", unix_ns: +m[1], ppb: +m[2], holdover_ms: +m[3], locked: m[4] === "1" };
  if ((m = PPSOUT_RE.exec(line)))
    return { t: "ppsout", unix_s: +m[1], late_us: +m[2], holdover_ms: +m[3] };
  return null;
}

// --- 最小 WebSocket (server→client text frame のみ) ---
const WS_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

function wsFrame(data: string): Buffer {
  const payload = Buffer.from(data, "utf8");
  const len = payload.length;
  let header: Buffer;
  if (len < 126) {
    header = Buffer.from([0x81, len]);
  } else if (len < 65536) {
    header = Buffer.alloc(4);
    header[0] = 0x81;
    header[1] = 126;
    header.writeUInt16BE(len, 2);
  } else {
    header = Buffer.alloc(10);
    header[0] = 0x81;
    header[1] = 127;
    header.writeUInt32BE(0, 2);
    header.writeUInt32BE(len, 6);
  }
  return Buffer.concat([header, payload]);
}

const clients = new Set<Socket>();

function broadcast(msg: Msg): void {
  const frame = wsFrame(JSON.stringify(msg));
  for (const sock of clients) {
    if (sock.writable) sock.write(frame);
  }
}

// 直近状態を保持し、新規接続クライアントに即送る (履歴の簡易リプレイ)。
let lastStatus: Msg = { t: "status", source: "?", connected: false };
// FW バージョンは起動時に 1 回だけ来るので、後から繋ぐクライアントにも送れるよう覚えておく。
let lastFw: Msg | null = null;

function handleUpgrade(req: http.IncomingMessage, socket: Socket): void {
  const key = req.headers["sec-websocket-key"];
  if (!key) {
    socket.destroy();
    return;
  }
  const accept = createHash("sha1").update(key + WS_MAGIC).digest("base64");
  socket.write(
    "HTTP/1.1 101 Switching Protocols\r\n" +
      "Upgrade: websocket\r\n" +
      "Connection: Upgrade\r\n" +
      `Sec-WebSocket-Accept: ${accept}\r\n\r\n`,
  );
  clients.add(socket);
  socket.write(wsFrame(JSON.stringify(lastStatus)));
  if (lastFw) socket.write(wsFrame(JSON.stringify(lastFw)));

  socket.on("data", (buf: Buffer) => {
    // クライアントからの close フレーム (opcode 0x8) だけ処理。
    if (buf.length > 0 && (buf[0] & 0x0f) === 0x8) {
      clients.delete(socket);
      socket.end();
    }
  });
  const drop = () => clients.delete(socket);
  socket.on("close", drop);
  socket.on("error", drop);
}

// --- 静的ファイル配信 ---
const PUBLIC_DIR = path.join(__dirname, "..", "public");
const MIME: Record<string, string> = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".map": "application/json",
};

function serveStatic(req: http.IncomingMessage, res: http.ServerResponse): void {
  const urlPath = (req.url || "/").split("?")[0];
  const rel = urlPath === "/" ? "index.html" : urlPath.replace(/^\/+/, "");
  const filePath = path.join(PUBLIC_DIR, rel);
  // PUBLIC_DIR の外には出さない。
  if (!filePath.startsWith(PUBLIC_DIR)) {
    res.writeHead(403).end("forbidden");
    return;
  }
  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404).end("not found");
      return;
    }
    res.writeHead(200, { "Content-Type": MIME[path.extname(filePath)] || "application/octet-stream" });
    res.end(data);
  });
}

// --- データソース ---
interface Args {
  port: number;
  chip: string;
  elf: string;
  replay: string | null;
  attach: boolean;
  log: string | null;
}

function parseArgs(argv: string[]): Args {
  const a: Args = {
    port: 8137,
    chip: "RP2040",
    elf: "../target/thumbv6m-none-eabi/debug/pico-gnss",
    replay: null,
    attach: false,
    log: null,
  };
  for (let i = 0; i < argv.length; i++) {
    const v = argv[i];
    if (v === "--port") a.port = +argv[++i];
    else if (v === "--chip") a.chip = argv[++i];
    else if (v === "--elf") a.elf = argv[++i];
    else if (v === "--replay") a.replay = argv[++i];
    else if (v === "--attach") a.attach = true;
    else if (v === "--log") a.log = argv[++i];
  }
  return a;
}

function feedLine(line: string): void {
  const msg = parseLine(line);
  if (!msg) return;
  if (msg.t === "fw") lastFw = msg;
  broadcast(msg);
}

/** probe-rs を起動し、stdout 各行を解析して配信する (実機モード)。 */
function startProbeRs(args: Args): void {
  const sub = args.attach ? "attach" : "run";
  const prArgs = [sub, "--chip", args.chip, args.elf];
  lastStatus = { t: "status", source: `probe-rs ${sub}`, connected: false };
  console.log(`[bridge] spawning: probe-rs ${prArgs.join(" ")}`);
  let child: ChildProcessWithoutNullStreams;
  try {
    child = spawn("probe-rs", prArgs);
  } catch (e) {
    console.error("[bridge] failed to spawn probe-rs:", e);
    lastStatus = { t: "status", source: "probe-rs", connected: false, note: String(e) };
    return;
  }
  // --log 指定時は probe-rs の生行をファイルにも書く (配信と同時にキャプチャ)。
  const logStream = args.log ? fs.createWriteStream(args.log, { flags: "a" }) : null;
  if (logStream) console.log(`[bridge] logging raw lines to ${args.log}`);
  const onLine = (line: string) => {
    if (logStream && (line.includes("NMEA ") || line.includes("PPS count=") || line.includes("SYNC ") || line.includes("TIME ") || line.includes("PPSOUT ") || line.includes("FW "))) {
      logStream.write(line + "\n");
    }
    if (line.includes("NMEA ") || line.includes("PPS count=") || line.includes("SYNC ")) {
      lastStatus = { t: "status", source: `probe-rs ${sub}`, connected: true };
      broadcast(lastStatus);
    }
    feedLine(line);
  };
  readline.createInterface({ input: child.stdout }).on("line", onLine);
  readline.createInterface({ input: child.stderr }).on("line", (l) => {
    if (l.trim()) console.error("[probe-rs]", l);
  });
  child.on("exit", (code) => {
    console.error(`[bridge] probe-rs exited (code ${code})`);
    lastStatus = { t: "status", source: "probe-rs", connected: false, note: `exited ${code}` };
    broadcast(lastStatus);
  });
}

/** sample.log を秒ペースで再生する (ハード不要モード)。PPS 行ごとに ~1s 待つ。 */
async function startReplay(file: string): Promise<void> {
  const abs = path.resolve(file);
  lastStatus = { t: "status", source: `replay ${path.basename(abs)}`, connected: true };
  console.log(`[bridge] replay from ${abs} (loops)`);
  for (;;) {
    const lines = fs.readFileSync(abs, "utf8").split(/\r?\n/).filter((l) => l.length > 0);
    if (lines.length === 0) {
      console.error("[bridge] replay file empty");
      return;
    }
    for (const line of lines) {
      feedLine(line);
      // PPS 行で 1 秒間隔を作る (実機の秒ペースを模す)。
      if (line.includes("PPS count=")) await sleep(1000);
    }
  }
}

// --- 起動 ---
const args = parseArgs(process.argv.slice(2));
const server = http.createServer(serveStatic);
server.on("upgrade", handleUpgrade);
server.listen(args.port, () => {
  console.log(`[bridge] http://localhost:${args.port}  (source: ${args.replay ? "replay" : args.attach ? "probe-rs attach" : "probe-rs run"})`);
  if (args.replay) {
    void startReplay(args.replay);
  } else {
    startProbeRs(args);
  }
});
