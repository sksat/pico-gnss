export const meta = {
  name: 'analyze-ctrlsweep-prbs',
  description: 'Evaluate the hardware controller-sweep PRBS capture: per-controller h[k], multi-lens, go/no-go',
  phases: [
    { title: 'Extract', detail: 'parse PPSGEN telemetry → per-controller h[k] + steady stats' },
    { title: 'Evaluate', detail: 'parallel lenses: resonance, recovery, effort, confound, trap' },
    { title: 'Synthesize', detail: 'reception-independent go/no-go per controller' },
  ],
}

// args = { log: "..." }; default is the interleave+kick capture.
const LOG = (args && args.log) || 'logs/pps-interleave-kick.log'

const PRIVACY = `CRITICAL PRIVACY RULE: ${LOG} is a raw RTT log that interleaves NMEA sentences containing the user's real GPS COORDINATES. You must ONLY read/parse lines that contain "PPSGEN" or "CTRLSWEEP". You must NEVER read, echo, quote, summarise, or reason about any NMEA line ($GP../$GN..) or any latitude/longitude/location value. Your output must contain ZERO location data. Prefer the provided python script (docs/report/analyze_prbs.py) and \`grep PPSGEN\`/\`grep CTRLSWEEP\` over reading the file raw. Do not copy the log anywhere; it is gitignored and must stay so.`

const CONTEXT = `Hardware GPSDO controller-sweep self-identification experiment (RP2040), INTERLEAVE + KICK edition. The firmware cycles selectable phase controllers (gnssdo::Controller) in one boot — cidx field = controller index: 0=pll_prod (production PID+Smith), 1=ab_boost (α-β + transient boost), 2=integ_rework, 3=pll_smith128, 4=naive_pid. Two improvements over the first pass: (1) INTERLEAVE — controllers run in a Latin-square schedule (each cidx in 5 short ~300-edge segments at different times/orders) so the order↔time-drift confound is broken; analyze_prbs.py pools segments per cidx. (2) KICK — while locked, every 150 locked edges a +8000 ns step disturbance is injected into the output phase (kick_ns field = 8000 on the kick edge, 0 otherwise; it appears in hwphase ~2 edges later due to output-FIFO latency). The 8000 ns kick exceeds outlier_ns(3000) so each controller outlier-rejects ~12 edges then accepts it, and it is large enough to briefly unlock the loop → this exercises ab_boost's transient BOOST (which fires during reacquisition), the thing the steady PRBS could not test. TWO reception-independent measurements: (a) steady resonance h[k] = crosscorr(inj=±96ns PRBS, hwphase) via docs/report/analyze_prbs.py (now kick-aware: it excludes the 30-edge post-kick window so the recovery transient does not contaminate the steady h[k]); (b) transient recovery via docs/report/analyze_kick_recovery.py — settle edges to return to the lock window after each kick, peak, and ab_boost boost-fire %. Both are reception-independent: PRBS/kick are known signals, reception noise is uncorrelated. The measurement trap still bars steady-offset/accuracy claims (hwphase = output vs the SAME receiver) — only h[k] resonance and kick-recovery dynamics are valid comparators. PPSGEN fields: count, hwphase_ns (control error), trim_mppb, p_ns, lk (locked 0/1), cidx, state (ab_boost 1=boosting), inj_ns (PRBS), kick_ns (8000 on kick edge), jit, rxbad. ${PRIVACY}`

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
          boost_duty_pct: { type: 'number', description: 'for ab_boost: % steady edges with state=1, else 0' },
          n_kicks: { type: 'integer', description: 'kick recovery events for this controller' },
          kick_settle_median: { type: 'number', description: 'median edges to resettle after a kick (from analyze_kick_recovery.py)' },
          kick_settle_p90: { type: 'number' },
          kick_boost_fire_pct: { type: 'number', description: 'for ab_boost: % of kick recoveries where boost fired' },
          median_jit: { type: 'number' }, rxbad_frac: { type: 'number' },
        },
        required: ['name', 'cidx', 'locked_prbs_samples', 'segments', 'h_peak', 'h_min', 'ring_zc', 'hk_head', 'steady_hwphase_sd_ns', 'n_kicks', 'kick_settle_median'],
      },
    },
  },
  required: ['ok', 'summary', 'per_controller'],
}
const extract = await agent(
  `${CONTEXT}\n\nExtract the per-controller comparison data from ${LOG}. Run BOTH:\n- \`python3 docs/report/analyze_prbs.py ${LOG}\` → steady resonance h[k] (peak/min/ring-zc per controller, kick-aware).\n- \`python3 docs/report/analyze_kick_recovery.py ${LOG}\` → transient recovery (n_kicks, median/p90 settle edges, ab_boost boost-fire %).\nAdditionally, with PPSGEN-only greps, for each controller: locked+PRBS sample count, number of sweep segments (interleave → expect up to 5), steady hwphase σ (locked edges, excluding the 120-edge post-switch warmup AND the 30-edge post-kick window), ab_boost steady boost duty (% state=1 outside kick recovery), reception balance (median jit, fraction rxbad>0). Report it all. Set ok=false (and say how much more capture time is needed) UNLESS all 5 controllers have ≥100 locked+PRBS samples AND ≥2 kick recovery events (modest samples — report the per-controller kick counts so the synthesis can hedge the recovery medians honestly).`,
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
  { key: 'recovery', q: `RECOVERY (now directly measured via the +8000ns kicks). Rank controllers by kick recovery: kick_settle_median (edges to return to the lock window after each kick; smaller = faster), kick_settle_p90, peak. THIS IS THE TEST OF ab_boost's REASON TO EXIST — its boost is meant to speed reacquisition. Check kick_boost_fire_pct: did ab_boost's boost actually fire during kick recoveries (it should; 8000ns >> innov_enter 3000ns and the kick unlocks the loop)? If it fired, is ab_boost's settle actually FASTER than the PI controllers? If ab_boost boosts but does NOT recover faster, that is a decisive no-go. Also confirm ab_boost's STEADY boost_duty stays ~0 (no false boosting between kicks). Give the recovery ranking with settle numbers and the boost-fire verdict.` },
  { key: 'effort', q: `STEADY CONTROL EFFORT (trap-limited — frame carefully). Compare steady hwphase σ and control effort per controller. REMEMBER the measurement trap: lower hwphase σ vs the same receiver is NOT proof of better true-UTC; a controller that tracks the receiver's wander harder looks 'better' on hwphase but may be worse vs UTC. State which differences are real-but-unverifiable vs artifacts.` },
  { key: 'confound', q: `CONFOUND / FAIRNESS CRITIC (adversarial). This pass INTERLEAVES (each controller in ~5 short segments at different times/orders) to break the first pass's order↔time-drift alias. Check whether it actually did: does each controller have multiple segments spread across the capture (segments field, ideally ~5), or did the capture not finish a full interleave cycle (then the alias is only partly broken)? Is reception balanced (median jit, rxbad_frac similar across controllers)? Are kick counts (n_kicks) comparable and did kicks land (peak ≈ 8000)? Were BOTH the post-switch warmup and the post-kick window excluded from steady h[k]? Is PRBS still linear (±96ns ≪ 3000ns)? List every remaining artifact risk and whether the interleave removed the order↔time confound.` },
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
  `Write the go/no-go report for the maintainer. PRIOR CONTEXT to reconcile: a first PRBS pass (cycle 1, single LONG 900-edge segments per controller, NO kicks, but a fixed controller order so order↔time-drift was fully aliased) ranked ab_boost WORST-damped (ring-zc 7) and pll_prod best (zc 0). THIS pass interleaves (breaks the order↔time alias) but uses SHORT 300-edge segments minus warmup minus kick-windows (~380 samples/controller vs ~800 in cycle 1), so its h[k] is noisier and the resonance ranking may not reproduce cycle 1 — if so, the honest reading is that the controllers' resonances are NOT robustly distinguishable on hardware (cycle-1's separation was partly confound/noise), NOT that the ranking reversed. The NEW thing this pass adds that cycle 1 could not: the +8000ns kicks finally FIRE ab_boost's boost, so its disturbance recovery is directly measured. Structure the report: (1) one-line bottom line — does any controller beat production reception-independently? (2) RECOVERY ranking from the kicks (settle edges + ab_boost boost-fire %) — the decisive new evidence; (3) RESONANCE: reconcile cycle-1 (cleaner, confounded) vs this (de-confounded, noisier) — state honestly whether ab_boost is robustly worse-damped or whether resonance is within noise; (4) explicit go/no-go for ab_boost using "adopt only if resonance ≤ production AND recovery improves" — note its boost now demonstrably fires yet recovery did/did-not improve; (5) trap-limited claims (need external reference); (6) remaining fairness caveats and whether more capture is needed. Be concise and honest; the host model over-stated ab_boost — demand hardware evidence. Output as markdown to append to docs/report/REPORT.md. ZERO coordinates/location data.`,
  { label: 'synthesize', phase: 'Synthesize' })

return { ready: true, per_controller: extract.per_controller, lenses, report: synth }
