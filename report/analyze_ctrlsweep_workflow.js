export const meta = {
  name: 'analyze-ctrlsweep-prbs',
  description: 'Evaluate the hardware controller-sweep PRBS capture: per-controller h[k], multi-lens, go/no-go',
  phases: [
    { title: 'Extract', detail: 'parse PPSGEN telemetry → per-controller h[k] + steady stats' },
    { title: 'Evaluate', detail: 'parallel lenses: resonance, recovery, effort, confound, trap' },
    { title: 'Synthesize', detail: 'reception-independent go/no-go per controller' },
  ],
}

// args = { log: "logs/pps-ctrlsweep-prbs.log" }
const LOG = (args && args.log) || 'logs/pps-ctrlsweep-prbs.log'

const PRIVACY = `CRITICAL PRIVACY RULE: ${LOG} is a raw RTT log that interleaves NMEA sentences containing the user's real GPS COORDINATES. You must ONLY read/parse lines that contain "PPSGEN" or "CTRLSWEEP". You must NEVER read, echo, quote, summarise, or reason about any NMEA line ($GP../$GN..) or any latitude/longitude/location value. Your output must contain ZERO location data. Prefer the provided python script (report/analyze_prbs.py) and \`grep PPSGEN\`/\`grep CTRLSWEEP\` over reading the file raw. Do not copy the log anywhere; it is gitignored and must stay so.`

const CONTEXT = `Hardware GPSDO controller-sweep self-identification experiment (RP2040). The firmware cycles selectable phase controllers (gnssdo::Controller) in one boot — cidx field = controller index: 0=pll_prod (production PID+Smith), 1=ab_boost (α-β + transient boost), 2=integ_rework, 3=pll_smith128, 4=naive_pid — switching every ~900 locked edges, and injects a known PRBS (inj_ns = ±96 ns, the 'inj_ns' field) into the output phase while locked. The closed-loop disturbance impulse response h[k] = crosscorr(inj, hwphase) cancels reception noise (uncorrelated with inj) and reveals each controller's closed-loop transfer (resonance / effective damping) reception-independently — the ONLY comparison valid under the measurement trap (hwphase = output vs the SAME receiver, so steady-state 'closer to UTC' is NOT verifiable without an external reference). PPSGEN fields: count, hwphase_ns (control error), trim_mppb (freq), p_ns (phase corr), lk (locked 0/1), cidx (controller), state (controller internal state; ab_boost 1=boosting), inj_ns (PRBS), jit (reception interval jitter), rxbad (reception quality). report/analyze_prbs.py already computes per-controller h[k] (peak/min/ring-zerocross). ${PRIVACY}`

phase('Extract')
const EXTRACT_SCHEMA = {
  type: 'object',
  properties: {
    ok: { type: 'boolean', description: 'true if there is enough locked+PRBS data for ≥2 controllers' },
    summary: { type: 'string' },
    per_controller: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          name: { type: 'string' },
          cidx: { type: 'integer' },
          locked_prbs_samples: { type: 'integer' },
          segments: { type: 'integer', description: 'how many sweep segments this controller got' },
          h_peak: { type: 'number' }, h_min: { type: 'number' }, ring_zc: { type: 'integer' },
          hk_head: { type: 'string', description: 'h[k] k=0..15 ×100, space-separated' },
          steady_hwphase_sd_ns: { type: 'number', description: 'σ of hwphase over locked non-PRBS-warmup edges' },
          boost_duty_pct: { type: 'number', description: 'for ab_boost: % edges with state=1, else 0' },
          median_jit: { type: 'number' }, rxbad_frac: { type: 'number' },
        },
        required: ['name', 'cidx', 'locked_prbs_samples', 'segments', 'h_peak', 'h_min', 'ring_zc', 'hk_head', 'steady_hwphase_sd_ns'],
      },
    },
  },
  required: ['ok', 'summary', 'per_controller'],
}
const extract = await agent(
  `${CONTEXT}\n\nExtract the per-controller comparison data from ${LOG}. Run \`python3 report/analyze_prbs.py ${LOG}\` for the h[k] table (peak/min/ring-zc per controller). Additionally, with PPSGEN-only greps, compute for each controller: locked+PRBS sample count, number of sweep segments, steady hwphase σ (locked edges, excluding the ~120-edge warmup after each switch), ab_boost boost duty (% state=1), and reception balance (median jit, fraction rxbad>0) so a later lens can check the order-vs-reception confound. Report it all. If fewer than 2 controllers have ≥100 locked+PRBS samples, set ok=false and say how much more capture time is needed (the sweep does ~900 locked edges ≈ 15 min per controller).`,
  { label: 'extract', phase: 'Extract', schema: EXTRACT_SCHEMA },
)

if (!extract || !extract.ok) {
  log(`Not enough data yet: ${extract ? extract.summary : 'extract failed'}`)
  return { ready: false, extract }
}

phase('Evaluate')
const LENS_SCHEMA = {
  type: 'object',
  properties: {
    finding: { type: 'string' },
    ranking: { type: 'string', description: 'controllers ranked on this lens, best first, with the numbers' },
    caveats: { type: 'string' },
  },
  required: ['finding', 'ranking', 'caveats'],
}
const DATA = JSON.stringify(extract.per_controller, null, 2)
const LENSES = [
  { key: 'resonance', q: `RESONANCE / DAMPING (the reception-independent decider). From each controller's h[k] (peak, min, ring-zerocross, hk_head): which is best-damped? A well-damped loop has h[k] peak near k≈1 then monotone decay; a more-negative min h, more zero-crossings, or longer ring-down = more underdamped resonance. Rank the controllers. Is ab_boost's resonance ≤ pll_prod's (the go condition)? Is any candidate clearly better-damped?` },
  { key: 'recovery', q: `RECOVERY. Using lock/relock transitions and (for ab_boost) the boost state, assess each controller's disturbance recovery. The sweep injects no explicit steps, so infer from natural relocks / boost engagements. Does ab_boost's boost engage appropriately (only on real transients, not steady wander — boost_duty should be LOW in steady)? Rank recovery where observable; say honestly if the capture cannot show it.` },
  { key: 'effort', q: `STEADY CONTROL EFFORT (trap-limited — frame carefully). Compare steady hwphase σ and control effort per controller. REMEMBER the measurement trap: lower hwphase σ vs the same receiver is NOT proof of better true-UTC; a controller that tracks the receiver's wander harder looks 'better' on hwphase but may be worse vs UTC. State which differences are real-but-unverifiable vs artifacts.` },
  { key: 'confound', q: `CONFOUND / FAIRNESS CRITIC (adversarial). Is this a FAIR reception-independent comparison? Check: did each controller get sampled across enough wall-clock/segments that order-vs-time-drift doesn't confound (ideally ≥2 segments each)? Is reception balanced across controllers (median jit, rxbad_frac similar)? Was the post-switch warmup excluded? Is the PRBS in the linear regime (±96ns ≪ ab_boost boost threshold 3000ns, so it shouldn't trigger boost)? List every reason a ranking here could be an artifact, and what additional capture would remove it.` },
  { key: 'trap', q: `MEASUREMENT-TRAP CRITIC. For each conclusion the other lenses might draw, classify it as (a) reception-independent & verifiable from h[k], or (b) trap-limited (needs an external reference: 2nd receiver/antenna, Rb, TIC). Be strict: only h[k]-based resonance/damping claims are truly reception-independent. Spell out what can and cannot be concluded from this capture alone.` },
]
const lensResults = await parallel(LENSES.map((L) => () =>
  agent(`${CONTEXT}\n\nPer-controller extracted data (JSON):\n${DATA}\n\nLENS — ${L.q}`,
    { label: `eval:${L.key}`, phase: 'Evaluate', schema: LENS_SCHEMA })
    .then((r) => ({ lens: L.key, ...r }))))
const lenses = lensResults.filter(Boolean)

phase('Synthesize')
const synth = await agent(
  `${CONTEXT}\n\nExtracted per-controller data:\n${DATA}\n\nLens analyses:\n${JSON.stringify(lenses, null, 2)}\n\n` +
  `Write the go/no-go report for the maintainer. Structure: (1) one-line bottom line — does any controller beat production reception-independently? (2) the per-controller h[k] resonance/damping ranking (the only reception-independent axis) with numbers; (3) explicit go/no-go for ab_boost using the rule "adopt only if h[k] resonance (min h, zero-cross, ring-down) ≤ production AND recovery improves"; (4) what is trap-limited and cannot be concluded without an external reference; (5) any fairness caveat (order/reception confound, segments per controller) and whether more capture is needed. Be concise and honest; the host model already over-stated ab_boost, so demand the hardware evidence. Output as markdown suitable to append to report/REPORT.md. Contain ZERO coordinates/location data.`,
  { label: 'synthesize', phase: 'Synthesize' })

return { ready: true, per_controller: extract.per_controller, lenses, report: synth }
