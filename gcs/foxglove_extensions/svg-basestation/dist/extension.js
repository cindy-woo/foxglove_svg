(() => {
"use strict";

// ─────────────────────────── SVG Basestation ──────────────────────────────────
//
// Ground-station panel for the SVG counter-UAS demonstration (Guard swarm vs.
// Strike swarm, see the project description). It is the SVG analogue of the
// DTC "Robot Control Panel" that anchors the foxglove_ws basestation layout:
// one panel that owns agent selection, the swarm-wide safety command, and the
// operator's health picture.
//
// It implements the two "Visual Insert Requirements" from the project brief:
//
//   1. CommLink Robustness Architecture
//      - multi-tiered topology showing dual-link redundancy: primary Wi-Fi
//        mesh with failover to 5G/4G cellular transport for DDS telemetry
//      - packet drop rate (<1% target), end-to-end RTT (<20 ms target),
//        PTP clock synchronization drift, link health state transitions
//
//   2. Battery & Power Management Telemetry
//      - per-agent SoC, operational voltage sag during high-rate maneuvers,
//        dynamic remaining mission time
//      - automated RTB thresholds from distance-to-pad energy calculations
//        (nominal >30%, conservative maneuver gating 20-30%, mandatory
//        failsafe landing <20%)
//
// Wiring matches robot/ros_ws/src/svg_ground_control (swarm_commander.py):
//   state     /{name}/odometry_conversion/odometry      nav_msgs/Odometry
//   battery   /{name}/fmu/out/battery_status            px4_msgs/BatteryStatus   (real)
//             /{name}/interface/mavros/battery          sensor_msgs/BatteryState (sim)
//   lifecycle /swarm_commander/{takeoff,start,hold,land,reset_fence}  std_srvs/Trigger
//   formation /svg/formation_command                    std_msgs/String
//
// Every link metric is DERIVED from the arrival statistics of the telemetry
// itself (no extra ROS node required); if a deployment publishes a real link
// report on {linkStatusTopicTemplate} (std_msgs/String JSON with any of
// drop_rate / rtt_ms / ptp_drift_ms / active_tier) those values override the
// derived ones. Fields with no source read "--" rather than showing a
// fabricated number.

// ─────────────────────────── constants ────────────────────────────────────────

const LIFECYCLE = [
  { id: "takeoff",     label: "Takeoff",     color: "#2563eb", confirm: true,
    hint: "Arm + offboard, ascend everyone to the scenario's initial positions, then HOLD" },
  { id: "start",       label: "Start",       color: "#10b981", confirm: true,
    hint: "Begin the scenario — nominal policies go live" },
  { id: "hold",        label: "HOLD ALL",    color: "#f59e0b", confirm: false,
    hint: "Panic button: every drone freezes at its current position" },
  { id: "land",        label: "Land All",    color: "#dc2626", confirm: true,
    hint: "Descend all commanded drones, disarm on touchdown" },
  { id: "reset_fence", label: "Reset Fence", color: "#6b7280", confirm: false,
    hint: "Clear a latched geofence breach" },
];

// Link health states, worst last — the swarm banner reports the worst one.
const LINK_STATE = {
  HEALTHY:  { rank: 0, label: "HEALTHY",  color: "#10b981" },
  FAILOVER: { rank: 1, label: "FAILOVER", color: "#3b82f6" },
  DEGRADED: { rank: 2, label: "DEGRADED", color: "#f59e0b" },
  LOST:     { rank: 3, label: "LOST",     color: "#dc2626" },
  NO_DATA:  { rank: 4, label: "NO DATA",  color: "#6b7280" },
};

// RTB verdicts, worst last.
const RTB_STATE = {
  NOMINAL:  { rank: 0, label: "NOMINAL",       color: "#10b981" },
  RTB_NOW:  { rank: 1, label: "RTB NOW",       color: "#3b82f6" },
  GATED:    { rank: 2, label: "MANEUVER GATED", color: "#f59e0b" },
  FAILSAFE: { rank: 3, label: "FAILSAFE LAND", color: "#dc2626" },
  NO_DATA:  { rank: 4, label: "NO DATA",       color: "#6b7280" },
};

const TIERS = [
  { id: "primary", label: "Wi-Fi Mesh",      sub: "primary DDS transport" },
  { id: "backup",  label: "5G / 4G Cellular", sub: "failover DDS transport" },
];

const METRIC_WINDOW_S = 10;    // sliding window for drop rate / RTT
const PTP_WINDOW_S = 120;      // sliding window for clock-offset drift slope
const LINK_LOSS_TIMEOUT_S = 1.0;
const MAX_TRANSITIONS = 60;
const SOC_SLOPE_WINDOW_S = 60; // sliding window for the burn-rate estimate
const UI_REFRESH_MS = 200;

// ─────────────────────────── defaults ─────────────────────────────────────────

const DEFAULTS = {
  drones: "drone_1,drone_2,drone_3",
  // Guard = defending swarm, Strike = intruding swarm (project scenarios A1/A2/C).
  roles: "guard,guard,strike",
  commanderNs: "/swarm_commander",
  formationTopic: "/svg/formation_command",
  stateTopicTemplate: "/{name}/odometry_conversion/odometry",
  primaryTopicTemplate: "/{name}/odometry_conversion/odometry",
  backupTopicTemplate: "/{name}/cellular/odometry",
  batteryTopicTemplate: "/{name}/fmu/out/battery_status",
  batteryAltTopicTemplate: "/{name}/interface/mavros/battery",
  linkStatusTopicTemplate: "/{name}/comms/link_status",
  padPosition: "0,0,0",
  positionOffsets: "",
  cruiseSpeedMps: 1.0,
  landSpeedMps: 0.3,
  reservePct: 8,
  rtbNominalPct: 30,
  rtbGatedPct: 20,
  dropTargetPct: 1.0,
  rttTargetMs: 20,
};

// ─────────────────────────── helpers ──────────────────────────────────────────

function splitList(s) {
  return String(s ?? "").split(",").map((x) => x.trim()).filter(Boolean);
}

function toSec(t) {
  if (t == null) return null;
  if (typeof t === "number") return t;
  const nanos = t.nanosec ?? t.nsec ?? 0;
  if (t.sec == null) return null;
  return Number(t.sec) + Number(nanos) * 1e-9;
}

function num(v) {
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

function clamp(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }

function median(arr) {
  if (!arr.length) return null;
  const s = [...arr].sort((a, b) => a - b);
  const m = s.length >> 1;
  return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2;
}

// Least-squares slope of y over x (used for PTP drift and SoC burn rate).
// x is centred first: these are epoch seconds (~1.7e9) and the uncentred normal
// equations lose the whole signal to float64 cancellation.
function slope(xs, ys) {
  const n = xs.length;
  if (n < 3) return null;
  const mx = xs.reduce((a, b) => a + b, 0) / n;
  const my = ys.reduce((a, b) => a + b, 0) / n;
  let sxx = 0, sxy = 0;
  for (let i = 0; i < n; i++) {
    const dx = xs[i] - mx;
    sxx += dx * dx;
    sxy += dx * (ys[i] - my);
  }
  if (sxx < 1e-9) return null;
  return sxy / sxx;
}

function fmt(v, digits, unit) {
  if (v == null || !Number.isFinite(v)) return "--";
  return v.toFixed(digits) + (unit ?? "");
}

function fmtDuration(sec) {
  if (sec == null || !Number.isFinite(sec) || sec < 0) return "--";
  const s = Math.round(sec);
  const m = Math.floor(s / 60), h = Math.floor(m / 60);
  const mm = String(m % 60).padStart(2, "0"), ss = String(s % 60).padStart(2, "0");
  return h > 0 ? `${h}:${mm}:${ss}` : `${m}:${ss}`;
}

function clockStamp(t) {
  return new Date(t * 1000).toLocaleTimeString();
}

// Worst (highest-ranked) state in the list; NO_DATA only when the list is empty.
function worst(states, table) {
  let out = null;
  for (const s of states) if (s && (out == null || s.rank > out.rank)) out = s;
  return out ?? table.NO_DATA;
}

function parseVec3(s, fallback) {
  const p = splitList(s).map(Number);
  if (p.length !== 3 || p.some((x) => !Number.isFinite(x))) return fallback;
  return p;
}

// ─────────────────────────── link statistics ──────────────────────────────────
//
// One instance per (agent, transport tier). Everything is derived from the
// arrival pattern of the DDS telemetry carried on that tier:
//
//   drop rate  — the nominal publish period P is the median inter-arrival gap;
//                a gap of k*P is counted as k-1 missed samples out of k expected.
//                Gaps longer than the loss timeout are outages, not loss, and
//                are excluded (the state machine reports those instead).
//   RTT        — 2 x the mean one-way delay, where the one-way delay is
//                rx_time - header.stamp. That identity holds exactly while PTP
//                keeps the publisher and the GCS on one timebase.
//   PTP drift  — the per-second minima of that same offset form the sync floor:
//                the residual clock error plus the minimum transit time. Its
//                least-squares slope over PTP_WINDOW_S is the drift rate, and a
//                ramping floor is the signal that the clocks are separating (so
//                the RTT above should be read against it).

function newTierStats(topic) {
  return {
    topic,
    lastRx: null,
    period: null,        // median inter-arrival, seconds
    gaps: [],            // [t, expected, missed]
    offsets: [],         // [t, offset_sec] (rx - stamp)
    dts: [],             // recent inter-arrival gaps for the median
    everSeen: false,
  };
}

function tierOnMessage(st, rxSec, stampSec) {
  st.everSeen = true;
  if (st.lastRx != null) {
    const dt = rxSec - st.lastRx;
    if (dt > 0) {
      st.dts.push(dt);
      if (st.dts.length > 200) st.dts.shift();
      // Robust period estimate from the gaps that look un-dropped.
      const med = median(st.dts) ?? dt;
      const clean = st.dts.filter((d) => d < 2.5 * med);
      st.period = median(clean) ?? med;
      if (dt <= LINK_LOSS_TIMEOUT_S) {
        const expected = Math.max(1, Math.round(dt / st.period));
        st.gaps.push([rxSec, expected, expected - 1]);
      }
      // A gap longer than the loss timeout is an outage, not packet loss: it is
      // already reported as LOST/FAILOVER and logged as a state transition, so
      // it is deliberately left out of the drop-rate window.
    }
  }
  st.lastRx = rxSec;
  if (stampSec != null) st.offsets.push([rxSec, rxSec - stampSec]);
  // Prune by age, not by sample count — a count cap would silently shorten the
  // PTP window on a fast topic.
  prune(st.gaps, rxSec - METRIC_WINDOW_S * 2);
  prune(st.offsets, rxSec - PTP_WINDOW_S * 1.5);
}

// Drop leading entries older than `cutoff` from an ascending [t, ...] list.
function prune(list, cutoff) {
  let i = 0;
  while (i < list.length && list[i][0] < cutoff) i++;
  if (i > 0) list.splice(0, i);
}

function tierDropRatePct(st, now) {
  const cutoff = now - METRIC_WINDOW_S;
  let expected = 0, missed = 0;
  for (let i = st.gaps.length - 1; i >= 0; i--) {
    if (st.gaps[i][0] < cutoff) break;
    expected += st.gaps[i][1];
    missed += st.gaps[i][2];
  }
  if (expected < 5) return null;
  return (missed / expected) * 100;
}

// End-to-end RTT = 2 x the mean one-way delay. With PTP holding the publisher
// and the GCS on the same timebase, (rx - header.stamp) *is* the one-way delay;
// the sync floor reported below is what tells the operator whether that
// assumption still holds.
function tierRttMs(st, now) {
  const cutoff = now - METRIC_WINDOW_S;
  const win = [];
  for (let i = st.offsets.length - 1; i >= 0; i--) {
    if (st.offsets[i][0] < cutoff) break;
    win.push(st.offsets[i][1]);
  }
  if (win.length < 5) return null;
  const mean = win.reduce((a, b) => a + b, 0) / win.length;
  return Math.max(0, mean * 2 * 1000);
}

// Returns { offsetMs, driftMsPerMin } — the PTP clock-sync error and its rate.
function tierPtp(st, now) {
  const cutoff = now - PTP_WINDOW_S;
  const xs = [], ys = [];
  for (let i = st.offsets.length - 1; i >= 0; i--) {
    if (st.offsets[i][0] < cutoff) break;
    xs.push(st.offsets[i][0]);
    ys.push(st.offsets[i][1]);
  }
  if (xs.length < 5) return { offsetMs: null, driftMsPerMin: null };
  // Bucket to per-second minima so queueing jitter doesn't pollute the slope.
  const buckets = new Map();
  for (let i = 0; i < xs.length; i++) {
    const k = Math.floor(xs[i]);
    const prev = buckets.get(k);
    if (prev == null || ys[i] < prev) buckets.set(k, ys[i]);
  }
  const keys = [...buckets.keys()].sort((a, b) => a - b);
  const bx = keys, by = keys.map((k) => buckets.get(k));
  const offsetMs = by[by.length - 1] * 1000;
  const s = slope(bx, by);
  return { offsetMs, driftMsPerMin: s == null ? null : s * 60 * 1000 };
}

// ─────────────────────────── battery normalisation ────────────────────────────
//
// Accepts px4_msgs/BatteryStatus (hardware, uXRCE-DDS) or
// sensor_msgs/BatteryState (sim, MAVROS) and flattens them to one shape.

function normaliseBattery(msg) {
  if (msg == null || typeof msg !== "object") return null;
  const out = { soc: null, voltage: null, vFiltered: null, current: null,
                timeRemaining: null, cells: null, warning: null };

  if (msg.voltage_v !== undefined || msg.remaining !== undefined) {
    // px4_msgs/BatteryStatus — "unknown" is encoded as -1 / 0 / NaN.
    const rem = num(msg.remaining);
    out.soc = rem != null && rem >= 0 ? rem * 100 : null;
    const v = num(msg.voltage_v);
    out.voltage = v != null && v > 0 ? v : null;
    const vf = num(msg.voltage_filtered_v);
    out.vFiltered = vf != null && vf > 0 ? vf : out.voltage;
    const c = num(msg.current_filtered_a ?? msg.current_a);
    out.current = c != null && c >= 0 ? c : null;
    const tr = num(msg.time_remaining_s);
    out.timeRemaining = tr != null && tr > 0 ? tr : null;
    const cc = num(msg.cell_count);
    out.cells = cc != null && cc > 0 ? cc : null;
    out.warning = num(msg.warning);
    return out;
  }

  if (msg.percentage !== undefined || msg.voltage !== undefined) {
    // sensor_msgs/BatteryState — current is negative while discharging.
    const p = num(msg.percentage);
    out.soc = p != null && p >= 0 ? (p <= 1.0001 ? p * 100 : p) : null;
    const v = num(msg.voltage);
    out.voltage = v != null && v > 0 ? v : null;
    out.vFiltered = out.voltage;
    const c = num(msg.current);
    out.current = c != null ? Math.abs(c) : null;
    out.cells = Array.isArray(msg.cell_voltage) && msg.cell_voltage.length
      ? msg.cell_voltage.length : null;
    return out;
  }
  return null;
}

// ─────────────────────────── per-agent runtime ────────────────────────────────

function newAgent(name, role, cfg) {
  return {
    name,
    role,                       // "guard" | "strike"
    tiers: {
      primary: newTierStats(cfg.primaryTopicTemplate.replace("{name}", name)),
      backup: newTierStats(cfg.backupTopicTemplate.replace("{name}", name)),
    },
    reported: null,             // last decoded {linkStatusTopicTemplate} report
    reportedAt: null,
    linkState: LINK_STATE.NO_DATA,
    linkSince: null,
    activeTier: null,
    transitions: [],            // [{t, from, to, tier}]
    // motion
    offset: [0, 0, 0],          // drone_position_offsets entry, added to odometry
    pos: null,                  // [x, y, z] world ENU
    speed: null,
    posAt: null,
    // battery
    batt: null,
    battAt: null,
    vRest: null,                // open-circuit baseline for the sag calculation
    sagPeak: 0,
    socHist: [],                // [t, soc] for the burn-rate estimate
  };
}

function noteLinkState(agent, next, tier, now) {
  if (agent.linkState === next) return;
  agent.transitions.push({
    t: now, from: agent.linkState.label, to: next.label, tier: tier ?? "--",
  });
  if (agent.transitions.length > MAX_TRANSITIONS) agent.transitions.shift();
  agent.linkState = next;
  agent.linkSince = now;
}

// Fold derived + reported link data into the agent's health state.
function evaluateLink(agent, cfg, now) {
  const r = agent.reported;
  const primary = agent.tiers.primary;
  const backup = agent.tiers.backup;

  const freshP = primary.lastRx != null && now - primary.lastRx <= LINK_LOSS_TIMEOUT_S;
  const freshB = backup.lastRx != null && now - backup.lastRx <= LINK_LOSS_TIMEOUT_S;

  let tier = r?.active_tier ?? (freshP ? "primary" : freshB ? "backup" : null);
  if (tier !== "primary" && tier !== "backup") tier = freshP ? "primary" : freshB ? "backup" : null;
  agent.activeTier = tier;

  const st = tier ? agent.tiers[tier] : primary;
  const derivedDrop = tierDropRatePct(st, now);
  const derivedRtt = tierRttMs(st, now);
  const ptp = tierPtp(st, now);

  agent.metrics = {
    dropPct: num(r?.drop_rate) != null ? Number(r.drop_rate) : derivedDrop,
    rttMs: num(r?.rtt_ms) != null ? Number(r.rtt_ms) : derivedRtt,
    ptpOffsetMs: num(r?.ptp_offset_ms) != null ? Number(r.ptp_offset_ms) : ptp.offsetMs,
    ptpDriftMsPerMin: num(r?.ptp_drift_ms) != null ? Number(r.ptp_drift_ms) : ptp.driftMsPerMin,
    primaryFresh: freshP,
    backupFresh: freshB,
    backupProvisioned: backup.everSeen,
    rateHz: st.period ? 1 / st.period : null,
  };

  let next;
  if (!primary.everSeen && !backup.everSeen) {
    next = LINK_STATE.NO_DATA;
  } else if (!freshP && !freshB) {
    next = LINK_STATE.LOST;
  } else {
    const m = agent.metrics;
    const badDrop = m.dropPct != null && m.dropPct > cfg.dropTargetPct;
    const badRtt = m.rttMs != null && m.rttMs > cfg.rttTargetMs;
    // Quality outranks routing: a link that failed over AND is out of spec is
    // reported as DEGRADED, with the tier column showing which path it took.
    next = badDrop || badRtt ? LINK_STATE.DEGRADED
      : !freshP && freshB ? LINK_STATE.FAILOVER
      : LINK_STATE.HEALTHY;
  }
  noteLinkState(agent, next, tier, now);
}

// Battery + distance-to-pad energy budget → RTB verdict.
function evaluatePower(agent, cfg, now) {
  const b = agent.batt;
  if (!b || b.soc == null) {
    agent.power = { state: RTB_STATE.NO_DATA };
    return;
  }

  // Burn rate (%/s) from the SoC slope; fall back to the autopilot estimate.
  const cutoff = now - SOC_SLOPE_WINDOW_S;
  const xs = [], ys = [];
  for (let i = agent.socHist.length - 1; i >= 0; i--) {
    if (agent.socHist[i][0] < cutoff) break;
    xs.push(agent.socHist[i][0]); ys.push(agent.socHist[i][1]);
  }
  let burnPctPerSec = null;
  const s = slope(xs, ys);
  if (s != null && s < 0) burnPctPerSec = -s;
  if (burnPctPerSec == null && b.timeRemaining) burnPctPerSec = b.soc / b.timeRemaining;

  // Dynamic remaining mission time.
  let missionTime = b.timeRemaining;
  if (missionTime == null && burnPctPerSec) missionTime = b.soc / burnPctPerSec;

  // Voltage sag: baseline is the highest voltage seen at low draw.
  const sag = agent.vRest != null && b.voltage != null
    ? Math.max(0, agent.vRest - b.voltage) : null;
  const sagPerCell = sag != null && b.cells ? sag / b.cells : null;

  // Distance-to-pad energy: cruise home, then descend.
  const pad = parseVec3(cfg.padPosition, [0, 0, 0]);
  let distance = null, returnTime = null, returnPct = null;
  if (agent.pos) {
    const dx = agent.pos[0] - pad[0], dy = agent.pos[1] - pad[1], dz = agent.pos[2] - pad[2];
    distance = Math.hypot(dx, dy);
    const cruise = Math.max(0.05, Number(cfg.cruiseSpeedMps) || 1);
    const land = Math.max(0.05, Number(cfg.landSpeedMps) || 0.3);
    returnTime = distance / cruise + Math.max(0, dz) / land;
    if (burnPctPerSec != null) returnPct = returnTime * burnPctPerSec + Number(cfg.reservePct);
  }

  const nominal = Number(cfg.rtbNominalPct);
  const gated = Number(cfg.rtbGatedPct);
  let state;
  if (b.soc < gated) state = RTB_STATE.FAILSAFE;
  else if (b.soc < nominal) state = RTB_STATE.GATED;
  else if (returnPct != null && b.soc <= returnPct) state = RTB_STATE.RTB_NOW;
  else state = RTB_STATE.NOMINAL;

  agent.power = {
    state, soc: b.soc, voltage: b.voltage, cells: b.cells,
    sag, sagPerCell, sagPeak: agent.sagPeak || null,
    current: b.current, missionTime, burnPctPerSec,
    distance, returnTime, returnPct,
    margin: returnPct != null ? b.soc - returnPct : null,
  };
}

// ─────────────────────────── styles ───────────────────────────────────────────
//
// Theme-neutral: colours inherit from Foxglove so the panel reads correctly in
// both the light and dark studio themes.

const STYLES = `
.sb-root {
  font-family: Inter, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  font-size: 12px; color: inherit; height: 100%; box-sizing: border-box;
  padding: 8px; overflow-y: auto; overflow-x: hidden;
  display: flex; flex-direction: column; gap: 8px; position: relative;
}
.sb-card {
  background: rgba(127,127,127,0.08); border: 1px solid rgba(127,127,127,0.28);
  border-radius: 6px; padding: 8px;
}
.sb-title {
  font-size: 11px; font-weight: 700; letter-spacing: 0.06em; text-transform: uppercase;
  opacity: 0.75; margin-bottom: 6px; padding-bottom: 4px;
  border-bottom: 1px solid rgba(127,127,127,0.28);
}
.sb-sub { font-size: 10px; opacity: 0.6; font-weight: 500; text-transform: none; letter-spacing: 0; }

/* banner */
.sb-banner { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.sb-banner .sb-spacer { flex: 1; }
.sb-chip {
  display: inline-flex; align-items: center; gap: 5px; padding: 3px 8px;
  border-radius: 999px; font-size: 11px; font-weight: 700; color: #fff; white-space: nowrap;
}
.sb-chip.sb-quiet { background: transparent; border: 1px solid rgba(127,127,127,0.4); color: inherit; font-weight: 600; }
.sb-dot { width: 8px; height: 8px; border-radius: 50%; flex-shrink: 0; }

/* command strip */
.sb-cmd-row { display: flex; gap: 6px; flex-wrap: wrap; align-items: center; }
.sb-btn {
  padding: 7px 12px; border: none; border-radius: 5px; color: #fff; cursor: pointer;
  font-size: 12px; font-weight: 700; letter-spacing: 0.02em;
}
.sb-btn:active { transform: scale(0.98); }
.sb-btn:disabled { opacity: 0.45; cursor: not-allowed; }
.sb-input {
  padding: 5px 7px; border-radius: 4px; border: 1px solid rgba(127,127,127,0.5);
  background: transparent; color: inherit; font-size: 12px; min-width: 0;
}
.sb-status { font-family: ui-monospace, monospace; font-size: 11px; opacity: 0.8; min-height: 14px; }

/* layout */
.sb-columns { display: grid; grid-template-columns: minmax(180px, 220px) minmax(0, 1fr); gap: 8px; align-items: start; }
@media (max-width: 640px) { .sb-columns { grid-template-columns: 1fr; } }
.sb-col { min-width: 0; display: flex; flex-direction: column; gap: 8px; }

/* roster */
.sb-agent {
  display: flex; flex-direction: column; gap: 4px; padding: 6px; border-radius: 5px;
  border: 1px solid rgba(127,127,127,0.3); cursor: pointer; background: transparent;
  color: inherit; text-align: left; width: 100%; box-sizing: border-box;
}
.sb-agent + .sb-agent { margin-top: 5px; }
.sb-agent.sb-selected { border-color: #10b981; box-shadow: inset 0 0 0 1px #10b981; }
.sb-agent-top { display: flex; align-items: center; gap: 6px; }
.sb-agent-name { font-weight: 700; font-size: 12px; flex: 1; }
.sb-role { font-size: 9px; font-weight: 800; letter-spacing: 0.06em; padding: 1px 5px; border-radius: 3px; color: #fff; }
.sb-role.guard { background: #2563eb; }
.sb-role.strike { background: #b45309; }

/* bars */
.sb-bar { position: relative; height: 8px; border-radius: 4px; background: rgba(127,127,127,0.25); overflow: hidden; }
.sb-bar-fill { position: absolute; left: 0; top: 0; bottom: 0; border-radius: 4px; transition: width 0.2s linear; }
.sb-bar-tick { position: absolute; top: -2px; bottom: -2px; width: 1px; background: rgba(255,255,255,0.75); box-shadow: 0 0 0 1px rgba(0,0,0,0.35); }
.sb-bar-lg { height: 14px; border-radius: 4px; }

/* metrics table */
.sb-table { width: 100%; border-collapse: collapse; font-size: 11px; font-variant-numeric: tabular-nums; }
.sb-table th {
  text-align: right; font-weight: 600; opacity: 0.6; padding: 3px 6px; white-space: nowrap;
  border-bottom: 1px solid rgba(127,127,127,0.3); font-size: 10px; text-transform: uppercase; letter-spacing: 0.04em;
}
.sb-table th:first-child, .sb-table td:first-child { text-align: left; }
.sb-table td { text-align: right; padding: 3px 6px; white-space: nowrap; border-bottom: 1px solid rgba(127,127,127,0.14); }
.sb-table tr.sb-selected td { background: rgba(16,185,129,0.12); }
.sb-ok { color: #10b981; }
.sb-warn { color: #f59e0b; font-weight: 700; }
.sb-bad { color: #dc2626; font-weight: 700; }
.sb-muted { opacity: 0.45; }

/* transitions log */
.sb-log {
  font-family: ui-monospace, monospace; font-size: 10.5px; line-height: 1.5;
  max-height: 108px; overflow-y: auto; background: rgba(0,0,0,0.16);
  border: 1px solid rgba(127,127,127,0.28); border-radius: 4px; padding: 5px 7px; white-space: pre-wrap;
}

/* power cards */
.sb-power-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(230px, 1fr)); gap: 8px; }
.sb-power { border: 1px solid rgba(127,127,127,0.3); border-radius: 5px; padding: 7px; display: flex; flex-direction: column; gap: 6px; }
.sb-power.sb-selected { border-color: #10b981; }
.sb-kv { display: flex; justify-content: space-between; gap: 8px; font-size: 11px; font-variant-numeric: tabular-nums; }
.sb-kv span:first-child { opacity: 0.6; }
.sb-note { font-size: 10px; opacity: 0.55; line-height: 1.45; }

/* confirm dialog */
.sb-overlay {
  position: absolute; inset: 0; background: rgba(0,0,0,0.55); display: flex;
  align-items: center; justify-content: center; z-index: 20; padding: 16px;
}
.sb-modal {
  background: #1f2937; color: #f9fafb; border-radius: 8px; padding: 14px;
  max-width: 320px; display: flex; flex-direction: column; gap: 10px;
  box-shadow: 0 8px 28px rgba(0,0,0,0.5);
}
.sb-modal-title { font-weight: 700; font-size: 13px; }
.sb-modal-row { display: flex; gap: 8px; justify-content: flex-end; }
`;

// ─────────────────────────── topology drawing ─────────────────────────────────
//
// Multi-tiered network topology: GCS/DDS domain → {Wi-Fi mesh | 5G/4G cellular}
// → agents. The path actually carrying each agent's telemetry is drawn solid
// and coloured by link health; the standby path is dashed and dimmed, so a
// failover is legible at a glance.

function topologySvg(agents) {
  const n = Math.max(1, agents.length);
  const colW = 118;
  const w = Math.max(430, n * colW + 40);
  const h = 208;
  const gcsY = 24, laneY = 92, agentY = 176;
  const laneCx = [w / 2 - 118, w / 2 + 118];
  const ax = (i) => (w - (n - 1) * colW) / 2 + i * colW;

  const parts = [];
  parts.push(`<svg viewBox="0 0 ${w} ${h}" width="100%" height="${h}" style="max-width:100%;overflow:visible">`);
  parts.push(`<style>
    .tp-lbl{font:600 10px Inter,sans-serif;fill:currentColor;opacity:.85;text-anchor:middle}
    .tp-sub{font:500 8.5px Inter,sans-serif;fill:currentColor;opacity:.5;text-anchor:middle}
    .tp-box{fill:rgba(127,127,127,.12);stroke:rgba(127,127,127,.55);stroke-width:1}
  </style>`);

  // GCS / DDS domain
  parts.push(`<rect x="${w / 2 - 92}" y="${gcsY - 15}" width="184" height="30" rx="6" class="tp-box"/>`);
  parts.push(`<text class="tp-lbl" x="${w / 2}" y="${gcsY - 1}">GCS · ROS 2 / DDS domain</text>`);
  parts.push(`<text class="tp-sub" x="${w / 2}" y="${gcsY + 10}">PTP time sync · QoS · dynamic routing</text>`);

  // Transport lanes
  TIERS.forEach((tier, ti) => {
    const anyActive = agents.some((a) => a.activeTier === tier.id);
    const cx = laneCx[ti];
    const stroke = anyActive ? "#10b981" : "rgba(127,127,127,.55)";
    parts.push(`<rect x="${cx - 88}" y="${laneY - 15}" width="176" height="30" rx="15"
      fill="${anyActive ? "rgba(16,185,129,.14)" : "rgba(127,127,127,.08)"}" stroke="${stroke}" stroke-width="${anyActive ? 1.6 : 1}"/>`);
    parts.push(`<text class="tp-lbl" x="${cx}" y="${laneY - 1}">${tier.label}</text>`);
    parts.push(`<text class="tp-sub" x="${cx}" y="${laneY + 10}">${tier.sub}</text>`);
    // GCS → lane
    parts.push(`<path d="M ${w / 2} ${gcsY + 15} C ${w / 2} ${laneY - 40}, ${cx} ${laneY - 45}, ${cx} ${laneY - 15}"
      fill="none" stroke="${stroke}" stroke-width="${anyActive ? 2 : 1}" ${anyActive ? "" : 'stroke-dasharray="4 3" opacity="0.5"'}/>`);
  });

  // Agents + their two links
  agents.forEach((a, i) => {
    const x = ax(i);
    const color = a.linkState.color;
    TIERS.forEach((tier, ti) => {
      const active = a.activeTier === tier.id;
      const fresh = tier.id === "primary" ? a.metrics?.primaryFresh : a.metrics?.backupFresh;
      const provisioned = tier.id === "primary" ? a.tiers.primary.everSeen : a.tiers.backup.everSeen;
      const cx = laneCx[ti];
      let stroke, width, dash, opacity;
      if (active) { stroke = color; width = 2.4; dash = ""; opacity = 1; }
      else if (fresh || provisioned) { stroke = "#9ca3af"; width = 1.2; dash = 'stroke-dasharray="5 4"'; opacity = 0.55; }
      else { stroke = "#6b7280"; width = 1; dash = 'stroke-dasharray="2 4"'; opacity = 0.28; }
      parts.push(`<path d="M ${cx} ${laneY + 15} C ${cx} ${agentY - 42}, ${x} ${agentY - 46}, ${x} ${agentY - 16}"
        fill="none" stroke="${stroke}" stroke-width="${width}" ${dash} opacity="${opacity}"/>`);
    });
    const roleColor = a.role === "strike" ? "#b45309" : "#2563eb";
    parts.push(`<rect x="${x - 46}" y="${agentY - 16}" width="92" height="32" rx="5"
      fill="rgba(127,127,127,.12)" stroke="${color}" stroke-width="1.6"/>`);
    parts.push(`<circle cx="${x - 34}" cy="${agentY - 4}" r="4" fill="${roleColor}"/>`);
    parts.push(`<text class="tp-lbl" x="${x + 5}" y="${agentY - 1}">${a.name}</text>`);
    parts.push(`<text class="tp-sub" x="${x}" y="${agentY + 11}">${a.linkState.label}</text>`);
  });

  parts.push(`</svg>`);
  return parts.join("");
}

// ─────────────────────────── panel ────────────────────────────────────────────

function activate(extensionContext) {
  extensionContext.registerPanel({
    name: "SVG Basestation",
    initPanel: (panelContext) => {

      // ── state ────────────────────────────────────────────────────────────
      const persisted = panelContext.initialState ?? {};
      const cfg = { ...DEFAULTS };
      for (const k of Object.keys(DEFAULTS)) {
        if (persisted[k] !== undefined && persisted[k] !== null) cfg[k] = persisted[k];
      }
      let selected = persisted.selected ?? null;
      let formation = persisted.formation ?? "";

      let agents = [];                 // rebuilt whenever the roster config changes
      const byTopic = new Map();       // topic → [{agent, kind}]
      let statusText = "";
      let lastTopoKey = null;
      // "Now" is anchored on the newest receive time and advanced by real
      // elapsed time. Live, that is just the wall clock; during playback it
      // tracks bag time, and it keeps advancing when the data stops so link
      // staleness is still detected.
      let clockRx = null, clockWall = null;

      function nowSec() {
        const wall = Date.now() / 1000;
        return clockRx == null ? wall : clockRx + (wall - clockWall);
      }
      function persist() {
        panelContext.saveState({ ...cfg, selected, formation });
      }

      // ── roster / subscriptions ───────────────────────────────────────────
      function rebuildAgents() {
        const names = splitList(cfg.drones);
        const roles = splitList(cfg.roles);
        const prev = new Map(agents.map((a) => [a.name, a]));
        // drone_position_offsets, flat x,y,z per agent (see swarm_sim.yaml).
        const offsets = splitList(cfg.positionOffsets).map(Number);
        agents = names.map((name, i) => {
          const role = (roles[i] ?? "guard").toLowerCase() === "strike" ? "strike" : "guard";
          const at = (k) => (Number.isFinite(offsets[i * 3 + k]) ? offsets[i * 3 + k] : 0);
          const agent = prev.get(name) ?? newAgent(name, role, cfg);
          agent.role = role;
          agent.offset = [at(0), at(1), at(2)];
          return agent;
        });
        if (!agents.some((a) => a.name === selected)) selected = agents[0]?.name ?? null;
        rebuildSubscriptions();
        buildRoster();
      }

      function rebuildSubscriptions() {
        byTopic.clear();
        const add = (topic, agent, kind) => {
          if (!topic) return;
          if (!byTopic.has(topic)) byTopic.set(topic, []);
          byTopic.get(topic).push({ agent, kind });
        };
        for (const a of agents) {
          const t = (tmpl) => (tmpl ? String(tmpl).replace("{name}", a.name) : null);
          a.tiers.primary.topic = t(cfg.primaryTopicTemplate);
          a.tiers.backup.topic = t(cfg.backupTopicTemplate);
          add(t(cfg.stateTopicTemplate), a, "state");
          add(a.tiers.primary.topic, a, "primary");
          add(a.tiers.backup.topic, a, "backup");
          add(t(cfg.batteryTopicTemplate), a, "battery");
          add(t(cfg.batteryAltTopicTemplate), a, "battery");
          add(t(cfg.linkStatusTopicTemplate), a, "linkstatus");
        }
        panelContext.subscribe([...byTopic.keys()].map((topic) => ({ topic })));
      }

      // ── message handling ─────────────────────────────────────────────────
      function handleState(a, msg, rx) {
        const p = msg?.pose?.pose?.position;
        if (p && num(p.x) != null) {
          const o = a.offset;
          a.pos = [Number(p.x) + o[0], Number(p.y) + o[1], Number(p.z) + o[2]];
          a.posAt = rx;
        }
        const v = msg?.twist?.twist?.linear;
        if (v && num(v.x) != null) a.speed = Math.hypot(Number(v.x), Number(v.y), Number(v.z));
      }

      function handleBattery(a, msg, rx) {
        const b = normaliseBattery(msg);
        if (!b) return;
        a.batt = b;
        a.battAt = rx;
        // Open-circuit baseline: the highest terminal voltage seen this session
        // is the least-loaded sample we have, so it is the best OCV proxy.
        if (b.voltage != null) {
          if (a.vRest == null || b.voltage > a.vRest) a.vRest = b.voltage;
          const sag = Math.max(0, a.vRest - b.voltage);
          if (sag > a.sagPeak) a.sagPeak = sag;
        }
        if (b.soc != null) {
          a.socHist.push([rx, b.soc]);
          prune(a.socHist, rx - SOC_SLOPE_WINDOW_S * 2);
        }
      }

      function handleLinkStatus(a, msg, rx) {
        try {
          const raw = msg?.data;
          a.reported = typeof raw === "string" ? JSON.parse(raw) : (raw ?? msg);
          a.reportedAt = rx;
        } catch { /* a malformed report just leaves the derived metrics in place */ }
      }

      panelContext.onRender = (renderState, done) => {
        const frame = renderState.currentFrame;
        if (frame) {
          for (const evt of frame) {
            const entries = byTopic.get(evt.topic);
            if (!entries) continue;
            const rx = toSec(evt.receiveTime) ?? Date.now() / 1000;
            if (clockRx == null || rx > clockRx) { clockRx = rx; clockWall = Date.now() / 1000; }
            const stamp = toSec(evt.message?.header?.stamp);
            for (const { agent, kind } of entries) {
              if (kind === "state") handleState(agent, evt.message, rx);
              else if (kind === "battery") handleBattery(agent, evt.message, rx);
              else if (kind === "linkstatus") handleLinkStatus(agent, evt.message, rx);
              else tierOnMessage(agent.tiers[kind], rx, stamp);
            }
          }
        }
        done();
      };
      panelContext.watch("currentFrame");
      panelContext.watch("topics");

      // ── DOM ──────────────────────────────────────────────────────────────
      const root = panelContext.panelElement;
      root.classList.add("sb-root");
      const styleEl = document.createElement("style");
      styleEl.textContent = STYLES;
      root.appendChild(styleEl);

      const el = (tag, cls, text) => {
        const n = document.createElement(tag);
        if (cls) n.className = cls;
        if (text != null) n.textContent = text;
        return n;
      };

      // Banner
      const banner = el("div", "sb-card sb-banner");
      const linkChip = el("span", "sb-chip");
      const powerChip = el("span", "sb-chip");
      const guardChip = el("span", "sb-chip sb-quiet");
      const strikeChip = el("span", "sb-chip sb-quiet");
      const clockEl = el("span", "sb-status");
      banner.append(linkChip, powerChip, guardChip, strikeChip, el("div", "sb-spacer"), clockEl);
      root.appendChild(banner);

      // Command strip
      const cmdCard = el("div", "sb-card");
      cmdCard.appendChild(el("div", "sb-title", "Swarm Command"));
      const cmdRow = el("div", "sb-cmd-row");
      for (const item of LIFECYCLE) {
        const b = el("button", "sb-btn", item.label);
        b.style.background = item.color;
        b.title = item.hint;
        b.addEventListener("click", () => {
          if (item.confirm) {
            askConfirm(item.label, `${item.hint}.\n\nSend "${item.id}" to ${cfg.commanderNs}?`,
              () => callLifecycle(item.id));
          } else {
            callLifecycle(item.id);
          }
        });
        cmdRow.appendChild(b);
      }
      cmdCard.appendChild(cmdRow);

      const formRow = el("div", "sb-cmd-row");
      formRow.style.marginTop = "6px";
      const formLabel = el("span", null, "Formation:");
      formLabel.style.opacity = "0.65";
      const formInput = el("input", "sb-input");
      formInput.type = "text";
      formInput.placeholder = "profile name, or 'next'";
      formInput.value = formation;
      formInput.style.flex = "1";
      formInput.addEventListener("change", () => { formation = formInput.value.trim(); persist(); });
      const formBtn = el("button", "sb-btn", "Send");
      formBtn.style.background = "#4f46e5";
      formBtn.title = `Publish the profile name on ${cfg.formationTopic} (std_msgs/String) to retarget the swarm`;
      formBtn.addEventListener("click", () => sendFormation(formInput.value.trim()));
      formRow.append(formLabel, formInput, formBtn);
      cmdCard.appendChild(formRow);

      const statusEl = el("div", "sb-status");
      statusEl.style.marginTop = "5px";
      cmdCard.appendChild(statusEl);
      root.appendChild(cmdCard);

      // Columns
      const columns = el("div", "sb-columns");
      const leftCol = el("div", "sb-col");
      const rightCol = el("div", "sb-col");
      columns.append(leftCol, rightCol);
      root.appendChild(columns);

      // Roster
      const rosterCard = el("div", "sb-card");
      rosterCard.appendChild(el("div", "sb-title", "Agents"));
      const rosterBody = el("div");
      rosterCard.appendChild(rosterBody);
      leftCol.appendChild(rosterCard);
      const rosterRows = new Map();

      function buildRoster() {
        rosterBody.textContent = "";
        rosterRows.clear();
        if (!agents.length) {
          rosterBody.appendChild(el("div", "sb-note", "No agents configured — set the drone list in the panel settings."));
          return;
        }
        for (const a of agents) {
          const row = el("button", "sb-agent");
          const top = el("div", "sb-agent-top");
          const dot = el("span", "sb-dot");
          const name = el("span", "sb-agent-name", a.name);
          const role = el("span", `sb-role ${a.role}`, a.role === "strike" ? "STRIKE" : "GUARD");
          top.append(dot, name, role);
          const bar = el("div", "sb-bar");
          const fill = el("div", "sb-bar-fill");
          bar.appendChild(fill);
          const meta = el("div", "sb-note");
          row.append(top, bar, meta);
          row.addEventListener("click", () => { selected = a.name; persist(); render(); });
          rosterBody.appendChild(row);
          rosterRows.set(a.name, { row, dot, fill, meta });
        }
      }

      // CommLink section
      const commCard = el("div", "sb-card");
      const commTitle = el("div", "sb-title");
      commTitle.append(document.createTextNode("CommLink Robustness "));
      commTitle.appendChild(el("span", "sb-sub", "— dual-link redundancy · drop rate · RTT · PTP drift"));
      commCard.appendChild(commTitle);
      const topoBox = el("div");
      topoBox.style.cssText = "margin-bottom:8px;overflow-x:auto;";
      commCard.appendChild(topoBox);
      const metricsTable = el("table", "sb-table");
      const metricsHead = el("thead");
      metricsHead.innerHTML =
        "<tr><th>Agent</th><th>Active tier</th><th>Rate</th><th>Drop</th><th>RTT</th>" +
        "<th>PTP sync floor</th><th>PTP drift</th><th>State</th></tr>";
      const metricsBody = el("tbody");
      metricsTable.append(metricsHead, metricsBody);
      commCard.appendChild(metricsTable);
      const targetsNote = el("div", "sb-note");
      targetsNote.style.marginTop = "5px";
      commCard.appendChild(targetsNote);
      commCard.appendChild(el("div", "sb-title", "Link health state transitions"));
      const logBox = el("div", "sb-log");
      commCard.appendChild(logBox);
      rightCol.appendChild(commCard);

      // Power section
      const powerCard = el("div", "sb-card");
      const powerTitle = el("div", "sb-title");
      powerTitle.append(document.createTextNode("Battery & Power Management "));
      powerTitle.appendChild(el("span", "sb-sub", "— SoC · voltage sag · mission time · RTB gating"));
      powerCard.appendChild(powerTitle);
      const powerGrid = el("div", "sb-power-grid");
      powerCard.appendChild(powerGrid);
      const powerNote = el("div", "sb-note");
      powerNote.style.marginTop = "6px";
      powerCard.appendChild(powerNote);
      rightCol.appendChild(powerCard);

      // Confirmation dialog
      let overlay = null;
      function askConfirm(title, message, onConfirm) {
        closeConfirm();
        overlay = el("div", "sb-overlay");
        const modal = el("div", "sb-modal");
        modal.appendChild(el("div", "sb-modal-title", title));
        const body = el("div", null, message);
        body.style.cssText = "font-size:12px;white-space:pre-wrap;line-height:1.45;";
        modal.appendChild(body);
        const row = el("div", "sb-modal-row");
        const cancel = el("button", "sb-btn", "Cancel");
        cancel.style.background = "#4b5563";
        cancel.addEventListener("click", closeConfirm);
        const ok = el("button", "sb-btn", "Confirm");
        ok.style.background = "#dc2626";
        ok.addEventListener("click", () => { closeConfirm(); onConfirm(); });
        row.append(cancel, ok);
        modal.appendChild(row);
        overlay.appendChild(modal);
        root.appendChild(overlay);
      }
      function closeConfirm() {
        if (overlay && overlay.parentNode) overlay.parentNode.removeChild(overlay);
        overlay = null;
      }

      // ── commands ─────────────────────────────────────────────────────────
      function setStatus(text) {
        statusText = `[${new Date().toLocaleTimeString()}] ${text}`;
        statusEl.textContent = statusText;
      }

      function callLifecycle(id) {
        const service = `${String(cfg.commanderNs).replace(/\/$/, "")}/${id}`;
        if (typeof panelContext.callService !== "function") {
          setStatus(`Service calls unavailable in this data source (wanted ${service})`);
          return;
        }
        setStatus(`Calling ${service} ...`);
        Promise.resolve()
          .then(() => panelContext.callService(service, {}))
          .then((res) => {
            const okFlag = res?.success;
            const msg = res?.message ? ` — ${res.message}` : "";
            setStatus(`${service}: ${okFlag === false ? "REJECTED" : "ok"}${msg}`);
          })
          .catch((err) => setStatus(`${service} failed: ${err?.message ?? err}`));
      }

      function sendFormation(nameArg) {
        const value = nameArg || formInput.value.trim();
        if (!value) { setStatus("Enter a formation profile name first"); return; }
        try {
          panelContext.advertise(cfg.formationTopic, "std_msgs/msg/String");
          panelContext.publish(cfg.formationTopic, { data: value });
          formation = value;
          persist();
          setStatus(`Formation "${value}" → ${cfg.formationTopic}`);
        } catch (err) {
          setStatus(`Formation publish failed: ${err?.message ?? err}`);
        }
      }

      // ── render ───────────────────────────────────────────────────────────
      function socColor(soc) {
        if (soc == null) return "#6b7280";
        if (soc < Number(cfg.rtbGatedPct)) return "#dc2626";
        if (soc < Number(cfg.rtbNominalPct)) return "#f59e0b";
        return "#10b981";
      }

      function gradeCell(td, value, target, digits, unit) {
        td.textContent = fmt(value, digits, unit);
        td.className = "";
        if (value == null) { td.classList.add("sb-muted"); return; }
        if (value <= target) td.classList.add("sb-ok");
        else if (value <= target * 2) td.classList.add("sb-warn");
        else td.classList.add("sb-bad");
      }

      function render() {
        const now = nowSec();
        for (const a of agents) {
          evaluateLink(a, cfg, now);
          evaluatePower(a, cfg, now);
        }

        // Banner
        const worstLink = worst(agents.map((a) => a.linkState), LINK_STATE);
        linkChip.textContent = `COMMLINK ${worstLink.label}`;
        linkChip.style.background = worstLink.color;
        const worstPower = worst(agents.map((a) => a.power?.state), RTB_STATE);
        powerChip.textContent = `POWER ${worstPower.label}`;
        powerChip.style.background = worstPower.color;
        const guards = agents.filter((a) => a.role === "guard").length;
        const strikes = agents.length - guards;
        guardChip.textContent = `Guard ${guards}`;
        strikeChip.textContent = `Strike ${strikes}`;
        const airborne = agents.filter((a) => a.pos && a.pos[2] > 0.3).length;
        clockEl.textContent = `${airborne}/${agents.length} airborne · ${clockStamp(now)}`;

        // Roster
        for (const a of agents) {
          const r = rosterRows.get(a.name);
          if (!r) continue;
          r.row.classList.toggle("sb-selected", a.name === selected);
          r.dot.style.background = a.linkState.color;
          const soc = a.power?.soc;
          r.fill.style.width = `${clamp(soc ?? 0, 0, 100)}%`;
          r.fill.style.background = socColor(soc);
          const rtt = a.metrics?.rttMs;
          r.meta.textContent =
            `${soc == null ? "-- %" : soc.toFixed(0) + "%"} · ` +
            `${a.linkState.label}${a.activeTier ? " (" + a.activeTier + ")" : ""} · ` +
            `${rtt == null ? "-- ms" : rtt.toFixed(0) + " ms"}`;
        }

        // Topology — reparsing SVG markup 5x/s is wasteful, so only redraw when
        // the picture would actually change.
        const topoKey = agents.map((a) =>
          `${a.name}:${a.role}:${a.linkState.label}:${a.activeTier}:` +
          `${a.tiers.primary.everSeen}${a.tiers.backup.everSeen}`).join("|");
        if (topoKey !== lastTopoKey) {
          lastTopoKey = topoKey;
          topoBox.innerHTML = agents.length
            ? topologySvg(agents)
            : '<div class="sb-note">No agents configured.</div>';
        }

        // Metrics table
        metricsBody.textContent = "";
        for (const a of agents) {
          const tr = el("tr");
          if (a.name === selected) tr.className = "sb-selected";
          const m = a.metrics ?? {};

          const tdName = el("td", null, a.name);
          const tdTier = el("td");
          if (a.activeTier === "backup") {
            tdTier.textContent = "5G / 4G";
            tdTier.className = "sb-warn";
          } else if (a.activeTier === "primary") {
            tdTier.textContent = "Wi-Fi mesh";
            tdTier.className = "sb-ok";
          } else {
            tdTier.textContent = "--";
            tdTier.className = "sb-muted";
          }
          const tdRate = el("td", m.rateHz == null ? "sb-muted" : null, fmt(m.rateHz, 1, " Hz"));
          const tdDrop = el("td");
          gradeCell(tdDrop, m.dropPct, Number(cfg.dropTargetPct), 2, " %");
          const tdRtt = el("td");
          gradeCell(tdRtt, m.rttMs, Number(cfg.rttTargetMs), 1, " ms");
          const tdOff = el("td", m.ptpOffsetMs == null ? "sb-muted" : null, fmt(m.ptpOffsetMs, 2, " ms"));
          const tdDrift = el("td");
          const drift = m.ptpDriftMsPerMin;
          tdDrift.textContent = drift == null ? "--" : `${drift >= 0 ? "+" : ""}${drift.toFixed(2)} ms/min`;
          tdDrift.className = drift == null ? "sb-muted"
            : Math.abs(drift) <= 1 ? "sb-ok" : Math.abs(drift) <= 5 ? "sb-warn" : "sb-bad";
          const tdState = el("td");
          const chip = el("span", "sb-chip", a.linkState.label);
          chip.style.background = a.linkState.color;
          chip.style.fontSize = "10px";
          tdState.appendChild(chip);

          tr.append(tdName, tdTier, tdRate, tdDrop, tdRtt, tdOff, tdDrift, tdState);
          tr.addEventListener("click", () => { selected = a.name; persist(); render(); });
          tr.style.cursor = "pointer";
          metricsBody.appendChild(tr);
        }
        if (!agents.length) {
          const tr = el("tr");
          const td = el("td", "sb-note", "No agents configured.");
          td.colSpan = 8;
          tr.appendChild(td);
          metricsBody.appendChild(tr);
        }

        const backupCount = agents.filter((a) => a.tiers.backup.everSeen).length;
        targetsNote.textContent =
          `Targets: packet drop < ${cfg.dropTargetPct}% · RTT < ${cfg.rttTargetMs} ms. ` +
          `Drop and RTT are derived from DDS telemetry arrival statistics over a ${METRIC_WINDOW_S}s window ` +
          `(RTT = 2x the mean of rx-time minus header stamp, which is the true one-way delay while PTP holds). ` +
          `The sync floor is the minimum of that offset and its slope over ${PTP_WINDOW_S}s is the PTP drift — ` +
          `a floor that ramps means the clocks are separating, so read RTT against it. ` +
          `Cellular failover transport: ${backupCount}/${agents.length} agents provisioned on ${cfg.backupTopicTemplate}.`;

        // Transition log — newest first, across the whole swarm.
        const events = [];
        for (const a of agents) {
          for (const t of a.transitions) events.push({ ...t, name: a.name });
        }
        events.sort((x, y) => y.t - x.t);
        logBox.textContent = events.length
          ? events.slice(0, 40).map((e) =>
              `${clockStamp(e.t)}  ${e.name.padEnd(10)} ${e.from} → ${e.to}  [${e.tier}]`).join("\n")
          : "No link state transitions recorded yet.";

        // Power cards
        powerGrid.textContent = "";
        for (const a of agents) {
          const p = a.power ?? { state: RTB_STATE.NO_DATA };
          const card = el("div", "sb-power");
          if (a.name === selected) card.classList.add("sb-selected");

          const head = el("div", "sb-agent-top");
          head.append(
            el("span", "sb-agent-name", a.name),
            el("span", `sb-role ${a.role}`, a.role === "strike" ? "STRIKE" : "GUARD"),
          );
          const rtbChip = el("span", "sb-chip", p.state.label);
          rtbChip.style.background = p.state.color;
          rtbChip.style.fontSize = "10px";
          head.appendChild(rtbChip);
          card.appendChild(head);

          // SoC bar with the RTB threshold ticks marked on it.
          const bar = el("div", "sb-bar sb-bar-lg");
          const fill = el("div", "sb-bar-fill");
          fill.style.width = `${clamp(p.soc ?? 0, 0, 100)}%`;
          fill.style.background = socColor(p.soc);
          bar.appendChild(fill);
          for (const pct of [Number(cfg.rtbGatedPct), Number(cfg.rtbNominalPct)]) {
            const tick = el("div", "sb-bar-tick");
            tick.style.left = `${clamp(pct, 0, 100)}%`;
            tick.title = `${pct}% threshold`;
            bar.appendChild(tick);
          }
          card.appendChild(bar);

          const kv = (k, v, cls) => {
            const row = el("div", "sb-kv");
            row.append(el("span", null, k), el("span", cls, v));
            card.appendChild(row);
          };
          kv("State of charge", p.soc == null ? "--" : `${p.soc.toFixed(1)} %`);
          kv("Voltage", p.voltage == null ? "--"
            : `${p.voltage.toFixed(2)} V${p.cells ? ` (${p.cells}S)` : ""}`);
          kv("Sag now / peak",
            p.sag == null ? "--"
              : `${p.sag.toFixed(2)} V / ${(p.sagPeak ?? 0).toFixed(2)} V` +
                (p.sagPerCell != null ? ` · ${p.sagPerCell.toFixed(3)} V/cell` : ""),
            p.sag != null && p.sagPerCell != null && p.sagPerCell > 0.15 ? "sb-warn" : null);
          kv("Draw", p.current == null ? "--" : `${p.current.toFixed(1)} A`);
          kv("Mission time left", fmtDuration(p.missionTime));
          kv("Distance to pad", p.distance == null ? "--" : `${p.distance.toFixed(1)} m`);
          kv("Return budget",
            p.returnPct == null ? "--"
              : `${p.returnPct.toFixed(1)} % (${fmtDuration(p.returnTime)})`);
          kv("Energy margin",
            p.margin == null ? "--" : `${p.margin >= 0 ? "+" : ""}${p.margin.toFixed(1)} %`,
            p.margin == null ? null : p.margin < 0 ? "sb-bad" : p.margin < 10 ? "sb-warn" : "sb-ok");

          if (p.state === RTB_STATE.FAILSAFE || p.state === RTB_STATE.GATED || p.state === RTB_STATE.RTB_NOW) {
            const act = el("button", "sb-btn", p.state === RTB_STATE.FAILSAFE ? "Failsafe: Land All" : "Land All");
            act.style.cssText = "background:" + p.state.color + ";padding:5px 10px;font-size:11px;";
            act.title = `${a.name}: ${p.state.label}. The commander lands the whole swarm — there is no per-agent land service.`;
            act.addEventListener("click", () =>
              askConfirm("Land All", `${a.name} is ${p.state.label}.\n\nLand every commanded drone?`,
                () => callLifecycle("land")));
            card.appendChild(act);
          }
          powerGrid.appendChild(card);
        }
        if (!agents.length) {
          powerGrid.appendChild(el("div", "sb-note", "No agents configured."));
        }

        powerNote.textContent =
          `RTB gating: nominal flight above ${cfg.rtbNominalPct}%, conservative maneuver gating ` +
          `${cfg.rtbGatedPct}-${cfg.rtbNominalPct}%, mandatory failsafe landing below ${cfg.rtbGatedPct}%. ` +
          `"RTB NOW" additionally fires when SoC drops to the distance-to-pad energy budget ` +
          `(cruise ${cfg.cruiseSpeedMps} m/s + descent ${cfg.landSpeedMps} m/s at the measured burn rate, ` +
          `plus a ${cfg.reservePct}% reserve). Sag is measured against the highest open-circuit voltage seen this session.`;

        statusEl.textContent = statusText;
      }

      // ── settings ─────────────────────────────────────────────────────────
      const NUMERIC = new Set([
        "cruiseSpeedMps", "landSpeedMps", "reservePct", "rtbNominalPct",
        "rtbGatedPct", "dropTargetPct", "rttTargetMs",
      ]);
      const ROSTER_KEYS = new Set([
        "drones", "roles", "stateTopicTemplate", "primaryTopicTemplate",
        "backupTopicTemplate", "batteryTopicTemplate", "batteryAltTopicTemplate",
        "linkStatusTopicTemplate", "positionOffsets",
      ]);

      function updateSettingsEditor() {
        panelContext.updatePanelSettingsEditor({
          actionHandler: (action) => {
            if (action.action !== "update") return;
            const key = action.payload.path[action.payload.path.length - 1];
            if (!(key in DEFAULTS)) return;
            cfg[key] = NUMERIC.has(key) ? Number(action.payload.value) : String(action.payload.value ?? "");
            persist();
            if (ROSTER_KEYS.has(key)) rebuildAgents();
            updateSettingsEditor();
            render();
          },
          nodes: {
            swarm: {
              label: "Swarm",
              fields: {
                drones: { label: "Agents", input: "string", value: cfg.drones,
                  help: "Comma-separated drone names, in drone_names order" },
                roles: { label: "Roles", input: "string", value: cfg.roles,
                  help: "Comma-separated guard|strike, one per agent" },
                commanderNs: { label: "Commander namespace", input: "string", value: cfg.commanderNs,
                  help: "std_srvs/Trigger lifecycle services live under this namespace" },
                formationTopic: { label: "Formation topic", input: "string", value: cfg.formationTopic },
              },
            },
            topics: {
              label: "Topics",
              fields: {
                stateTopicTemplate: { label: "State", input: "string", value: cfg.stateTopicTemplate },
                primaryTopicTemplate: { label: "Primary link (Wi-Fi mesh)", input: "string", value: cfg.primaryTopicTemplate },
                backupTopicTemplate: { label: "Backup link (5G/4G)", input: "string", value: cfg.backupTopicTemplate },
                batteryTopicTemplate: { label: "Battery (PX4)", input: "string", value: cfg.batteryTopicTemplate },
                batteryAltTopicTemplate: { label: "Battery (MAVROS)", input: "string", value: cfg.batteryAltTopicTemplate },
                linkStatusTopicTemplate: { label: "Link report (optional)", input: "string", value: cfg.linkStatusTopicTemplate,
                  help: "std_msgs/String JSON; drop_rate / rtt_ms / ptp_offset_ms / ptp_drift_ms / active_tier override the derived values" },
              },
            },
            comms: {
              label: "CommLink targets",
              fields: {
                dropTargetPct: { label: "Packet drop target (%)", input: "number", value: cfg.dropTargetPct, step: 0.1 },
                rttTargetMs: { label: "RTT target (ms)", input: "number", value: cfg.rttTargetMs, step: 1 },
              },
            },
            power: {
              label: "Power & RTB",
              fields: {
                padPosition: { label: "Landing pad (x,y,z)", input: "string", value: cfg.padPosition },
                positionOffsets: { label: "Position offsets", input: "string", value: cfg.positionOffsets,
                  help: "Flat x,y,z per agent, matching drone_position_offsets; blank = none" },
                cruiseSpeedMps: { label: "Cruise speed (m/s)", input: "number", value: cfg.cruiseSpeedMps, step: 0.1 },
                landSpeedMps: { label: "Land speed (m/s)", input: "number", value: cfg.landSpeedMps, step: 0.1 },
                reservePct: { label: "Reserve (%)", input: "number", value: cfg.reservePct, step: 1 },
                rtbNominalPct: { label: "Nominal above (%)", input: "number", value: cfg.rtbNominalPct, step: 1 },
                rtbGatedPct: { label: "Failsafe below (%)", input: "number", value: cfg.rtbGatedPct, step: 1 },
              },
            },
          },
        });
      }

      // ── boot ─────────────────────────────────────────────────────────────
      panelContext.setDefaultPanelTitle("SVG Basestation");
      rebuildAgents();
      updateSettingsEditor();
      render();

      const timer = setInterval(render, UI_REFRESH_MS);

      // Foxglove throttles JS while the browser tab is hidden, so latched /
      // TRANSIENT_LOCAL samples can be dropped from the queue during the gap.
      // Re-subscribing on resume replays them (same approach as robot-commands).
      const onVisibilityChange = () => {
        if (typeof document !== "undefined" && !document.hidden) {
          rebuildSubscriptions();
          render();
        }
      };
      if (typeof document !== "undefined") {
        document.addEventListener("visibilitychange", onVisibilityChange);
      }

      return () => {
        clearInterval(timer);
        if (typeof document !== "undefined") {
          document.removeEventListener("visibilitychange", onVisibilityChange);
        }
        closeConfirm();
        byTopic.clear();
        panelContext.subscribe([]);
        root.classList.remove("sb-root");
      };
    },
  });
}

module.exports = { activate };
})();
