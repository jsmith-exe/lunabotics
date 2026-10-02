// Entry point: telemetry stream, shared state, health/warnings, keys, render loop.
//
// The page is read-only with respect to the rover: it never sends commands.
// All data arrives on one Server-Sent Events stream from teleop_hud (/events);
// camera frames come as MJPEG from /cam/<name>.mjpg.

import { Surface, prefs, isNum, deg, fmtSigned, STALE_S, DEAD_S, TILT_CAUTION, TILT_CRIT } from "./util.js";
import { CameraDeck } from "./cameras.js";
import { TacticalMap } from "./map.js";
import { drawDrive, drawWheels, drawAttitude, drawImplement, drawTrace, TraceHistory } from "./gauges.js";

// ------------------------------------------------------------------ tuning
// Wheel stall: commanded speed above this, wheel surface speed below the other.
const STALL_CMD = 0.08;
const STALL_WHEEL = 0.02;
const STALL_HOLD_S = 0.8;
// A command counts as current for this long. Longer than the mux's 0.25 s
// timeout on purpose: nav_pub republishes at 5 Hz, so with any jitter the
// display would flicker between TELEOP and IDLE.
const CMD_HOLD_S = 0.5;
// Footprint closer than this to the arena wall raises a caution.
const WALL_CAUTION_M = 0.3;

// ------------------------------------------------------------------- state
const S = {
  connected: false,
  everConnected: false,
  lastMsg: 0,          // performance.now() of the last SSE message
  tele: null,
  config: null,
  costmap: null,
  plan: null,
  logs: [],
  derived: {},         // computed each tele frame: pose, frame name, stalls...
};

const $ = (id) => document.getElementById(id);
const trace = new TraceHistory(30, 20);
const cams = new CameraDeck(S);
const map = new TacticalMap($("map"), S);

const surfaces = [
  new Surface($("c-drive"), (ctx, w, h) => drawDrive(ctx, w, h, S)),
  new Surface($("c-wheels"), (ctx, w, h) => drawWheels(ctx, w, h, S)),
  new Surface($("c-att"), (ctx, w, h) => drawAttitude(ctx, w, h, S)),
  new Surface($("c-impl"), (ctx, w, h) => drawImplement(ctx, w, h, S)),
  new Surface($("c-trace"), (ctx, w, h) => drawTrace(ctx, w, h, S, trace)),
];

// ------------------------------------------------------------- SSE stream
let es = null;
function connect() {
  es = new EventSource("/events");
  es.addEventListener("open", () => {
    const wasDown = !S.connected;
    S.connected = true;
    S.everConnected = true;
    S.lastMsg = performance.now();
    if (wasDown) cams.reconnectAll();
  });
  es.addEventListener("error", () => {
    // EventSource retries by itself (retry: 1000 from the server).
    S.connected = false;
  });
  const on = (name, fn) =>
    es.addEventListener(name, (ev) => {
      S.lastMsg = performance.now();
      try {
        fn(JSON.parse(ev.data));
      } catch (e) {
        console.error(`bad ${name} message`, e);
      }
    });
  on("tele", onTele);
  on("config", (c) => {
    S.config = c;
    map.onConfig();
  });
  on("costmap", (c) => map.onCostmap(c));
  on("plan", (p) => (S.plan = p));
  on("log", onLog);
}

function onTele(t) {
  S.tele = t;
  derive(t);
  trace.push(t, S.derived);
  map.onTele();
  cams.onTele();
}

// ------------------------------------------------------- derived quantities
const stallSince = { fl: null, fr: null, rl: null, rr: null };

function topicAge(name) {
  const tp = S.tele?.topics?.find((x) => x.name === name);
  return tp && isNum(tp.age) ? tp.age : null;
}

function derive(t) {
  const d = S.derived;
  const fresh = (x) => x && isNum(x.age) && x.age < DEAD_S;
  d.jointsFresh = (topicAge("/joint_states") ?? 99) < DEAD_S;
  d.live = fresh(t.odom) || d.jointsFresh;
  // Prefer the map pose (aligned with the arena), then odom. If both have gone
  // stale keep the last one so the map can show where the rover was last seen.
  if (fresh(t.map)) [d.frame, d.pose] = ["map", t.map];
  else if (fresh(t.odom_pose)) [d.frame, d.pose] = ["odom", t.odom_pose];
  else if (t.map) [d.frame, d.pose] = ["map", t.map];
  else if (t.odom_pose) [d.frame, d.pose] = ["odom", t.odom_pose];
  else [d.frame, d.pose] = [null, null];
  d.poseStale = !!d.pose && !fresh(d.pose);
  // Roll/pitch: the local EKF is the smoother source.
  d.att = fresh(t.odom_pose) ? t.odom_pose : fresh(t.map) ? t.map : null;

  // The command the drive is actually following.
  const c = t.cmd || {};
  const live = (x) => x && isNum(x.age) && x.age < CMD_HOLD_S;
  d.cmd = live(c.teleop) ? c.teleop : live(c.nav) ? c.nav : null;
  d.cmdVx = d.cmd ? d.cmd.vx : 0;
  d.cmdWz = d.cmd ? d.cmd.wz : 0;

  // Measured motion.
  d.vx = t.odom && t.odom.age < DEAD_S ? t.odom.vx : null;
  d.wz = t.odom && t.odom.age < DEAD_S ? t.odom.wz : null;

  // Stall: a wheel on a side being driven that is not turning.
  const r = S.config?.rover?.wheel_radius ?? 0.15;
  const sep = S.config?.rover?.wheel_separation ?? 0.58;
  const wantL = d.cmdVx - (d.cmdWz * sep) / 2;
  const wantR = d.cmdVx + (d.cmdWz * sep) / 2;
  const now = performance.now() / 1000;
  const jointsFresh = (topicAge("/joint_states") ?? 99) < STALE_S;  // stricter: stall needs live data
  d.stalled = [];
  for (const k of ["fl", "fr", "rl", "rr"]) {
    const want = k.endsWith("l") ? wantL : wantR;
    const w = t.wheels?.[k];
    const stuck = jointsFresh && isNum(w) && Math.abs(want) > STALL_CMD && Math.abs(w * r) < STALL_WHEEL;
    if (stuck) {
      stallSince[k] ??= now;
      if (now - stallSince[k] > STALL_HOLD_S) d.stalled.push(k);
    } else stallSince[k] = null;
  }
}

// ------------------------------------------------------------------- logs
let logLevel = prefs.get("logLevel", 20);
const seenLog = new Set();
function onLog(entries) {
  let added = false;
  for (const e of entries) {
    if (seenLog.has(e.id)) continue;
    seenLog.add(e.id);
    S.logs.push(e);
    added = true;
    if (e.lvl >= 30) showTicker(e);
  }
  if (S.logs.length > 400) S.logs.splice(0, S.logs.length - 400);
  if (added) renderLog();
}

function renderLog() {
  const el = $("log");
  const stick = el.scrollTop + el.clientHeight >= el.scrollHeight - 8;
  const rows = S.logs.filter((e) => e.lvl >= logLevel).slice(-200);
  el.replaceChildren(
    ...rows.map((e) => {
      const ln = document.createElement("div");
      ln.className = `ln l${e.lvl}`;
      const tm = new Date(e.t * 1000);
      const ts = isNaN(tm) ? "" : tm.toTimeString().slice(0, 8);
      const t = document.createElement("span");
      t.className = "t";
      t.textContent = ts;
      const n = document.createElement("span");
      n.className = "n";
      n.textContent = e.node;
      ln.append(t, n, document.createTextNode(e.msg));
      return ln;
    }),
  );
  if (stick) el.scrollTop = el.scrollHeight;
}

let tickerTimer = null;
function showTicker(e) {
  const el = $("ticker");
  el.textContent = `${e.lvl >= 40 ? "ERROR" : "WARN"} · ${e.node}: ${e.msg}`;
  el.classList.toggle("err", e.lvl >= 40);
  el.classList.add("show");
  clearTimeout(tickerTimer);
  tickerTimer = setTimeout(() => el.classList.remove("show"), 6000);
}

for (const b of $("log-filters").querySelectorAll("button")) {
  b.classList.toggle("on", Number(b.dataset.l) === logLevel);
  b.addEventListener("click", () => {
    logLevel = Number(b.dataset.l);
    prefs.set("logLevel", logLevel);
    for (const o of $("log-filters").querySelectorAll("button")) o.classList.toggle("on", o === b);
    renderLog();
  });
}

// ------------------------------------------------------------- data links
function renderLinks() {
  const t = S.tele;
  if (!t) return;
  const rows = [...t.topics].sort((a, b) => a.kind.localeCompare(b.kind) || a.name.localeCompare(b.name));
  $("links").replaceChildren(
    ...rows.map((tp) => {
      const tr = document.createElement("tr");
      const never = tp.age === null;
      // Command/event topics are legitimately quiet when nobody is using
      // them; only continuous streams go red when they stop.
      const continuous = ["odom", "joints", "camera"].includes(tp.kind);
      const st = never ? "" : tp.age < STALE_S * 2 ? "ok" : !continuous ? "idle" : tp.age < 5 ? "stale" : "dead";
      if (never) tr.className = "never";
      const cells = [
        `<span class="led ${st}"></span>`,
        tp.name,
        never ? "—" : tp.hz.toFixed(1),
        never ? "never" : tp.age < 10 ? tp.age.toFixed(1) + "s" : ">10s",
        tp.bps ? (tp.bps / 1024).toFixed(1) : "",
      ];
      tr.innerHTML = cells.map((c) => `<td>${c}</td>`).join("");
      tr.children[1].textContent = tp.name; // topic names are data; keep them out of innerHTML
      return tr;
    }),
  );
}

// --------------------------------------------------------- header widgets
function verdict(el, state) {
  el.className = "magi-node " + state;
  el.querySelector(".mn-verdict").textContent =
    { ok: "承認", warn: "審議", fail: "否定", off: "—" }[state];
}

function renderHeader() {
  const t = S.tele;
  const d = S.derived;
  // Control source.
  const src = t ? t.source : "IDLE";
  const se = $("source");
  se.className = "source " + src.toLowerCase();
  se.querySelector(".source-v").textContent = src === "AUTO" ? "AUTONOMY" : src;

  // MAGI: LINK = data actually arriving from the rover.
  const odomAge = t?.odom?.age ?? null;
  const jsAge = topicAge("/joint_states");
  const freshest = Math.min(odomAge ?? 99, jsAge ?? 99);
  if (!S.connected) verdict($("magi-link"), "fail");
  else if (!t || freshest === 99) verdict($("magi-link"), "off");
  else verdict($("magi-link"), freshest < STALE_S ? "ok" : freshest < DEAD_S ? "warn" : "fail");

  // DRIVE = joint feedback alive and no stalled wheel.
  if (!t || jsAge === null) verdict($("magi-drive"), "off");
  else if (jsAge > DEAD_S) verdict($("magi-drive"), "fail");
  else verdict($("magi-drive"), d.stalled?.length || jsAge > STALE_S ? "warn" : "ok");

  // LOCALISE = a live map pose.
  if (!t) verdict($("magi-loc"), "off");
  else if (d.poseStale || !d.frame) verdict($("magi-loc"), "fail");
  else if (d.frame === "map") verdict($("magi-loc"), t.map.age < 0.5 ? "ok" : "warn");
  else verdict($("magi-loc"), "warn");

  $("zone").textContent = map.zoneName() ?? "—";
}

// --------------------------------------------------------------- warnings
let lastWarnKey = "";
let muted = prefs.get("muted", false);
function computeWarnings() {
  const t = S.tele;
  const d = S.derived;
  const w = [];
  if (!t) {
    if (S.connected) w.push({ lvl: "info", jp: "待機", txt: "AWAITING TELEMETRY", sub: "HUD up; nothing from teleop_hud yet" });
    return w;
  }
  const odomAge = t.odom?.age ?? null;
  const jsAge = topicAge("/joint_states");
  if (odomAge === null && jsAge === null) {
    w.push({ lvl: "info", jp: "待機", txt: "NO ROVER DATA YET", sub: "no /odometry/filtered or /joint_states" });
  } else if (Math.min(odomAge ?? 99, jsAge ?? 99) > DEAD_S) {
    const a = Math.min(odomAge ?? 99, jsAge ?? 99);
    w.push({ lvl: "crit", jp: "通信途絶", txt: "ROVER TELEMETRY LOST", sub: `last data ${a < 99 ? a.toFixed(1) + " s" : "—"} ago` });
  }

  const att = d.att;
  if (att && isNum(att.roll) && isNum(att.pitch)) {
    const tilt = Math.max(Math.abs(deg(att.roll)), Math.abs(deg(att.pitch)));
    if (tilt >= TILT_CRIT) w.push({ lvl: "crit", jp: "傾斜", txt: "TILT CRITICAL", sub: `roll ${fmtSigned(deg(att.roll), 0)}°  pitch ${fmtSigned(deg(att.pitch), 0)}°` });
    else if (tilt >= TILT_CAUTION) w.push({ lvl: "caution", jp: "傾斜", txt: "TILT", sub: `roll ${fmtSigned(deg(att.roll), 0)}°  pitch ${fmtSigned(deg(att.pitch), 0)}°` });
  }

  const wall = map.wallClearance();
  if (wall !== null) {
    if (wall < 0) w.push({ lvl: "crit", jp: "境界", txt: "OUTSIDE ARENA", sub: `${(-wall).toFixed(2)} m past the wall` });
    else if (wall < WALL_CAUTION_M) w.push({ lvl: "caution", jp: "境界", txt: "WALL PROXIMITY", sub: `${wall.toFixed(2)} m clearance` });
  }

  if (d.stalled?.length) w.push({ lvl: "caution", jp: "停止", txt: "WHEEL STALL", sub: d.stalled.map((k) => k.toUpperCase()).join(" ") + " not turning" });

  const c = t.cmd || {};
  if (d.cmd && Math.abs(d.cmdVx) + Math.abs(d.cmdWz) > 0.01 && (!c.out || c.out.age > 0.5)) {
    w.push({ lvl: "caution", jp: "遮断", txt: "MUX OUTPUT SILENT", sub: "commands sent, nothing on /diff_cont/cmd_vel_unstamped" });
  }

  if (d.live && d.frame === "odom") w.push({ lvl: "info", jp: "測位", txt: "NO MAP FIX", sub: "map → base TF missing; map shows odom" });
  return w;
}

// Browsers only allow audio after the operator has interacted with the page.
let audio = null;
const unlockAudio = () => {
  try {
    audio ??= new AudioContext();
    audio.resume();
  } catch {
    /* no audio device */
  }
};
window.addEventListener("pointerdown", unlockAudio, { once: true });
window.addEventListener("keydown", unlockAudio, { once: true });
function beep() {
  if (muted || !audio) return;
  try {
    const t0 = audio.currentTime;
    for (const [i, f] of [880, 660].entries()) {
      const o = audio.createOscillator();
      const g = audio.createGain();
      o.type = "square";
      o.frequency.value = f;
      g.gain.setValueAtTime(0.06, t0 + i * 0.14);
      g.gain.exponentialRampToValueAtTime(0.0001, t0 + i * 0.14 + 0.12);
      o.connect(g).connect(audio.destination);
      o.start(t0 + i * 0.14);
      o.stop(t0 + i * 0.14 + 0.13);
    }
  } catch {
    /* no audio device */
  }
}

let prevCrit = new Set();
function renderWarnings() {
  const w = computeWarnings();
  const key = w.map((x) => x.lvl + x.txt + x.sub).join("|");
  if (key !== lastWarnKey) {
    lastWarnKey = key;
    $("warnings").replaceChildren(
      ...w.map((x) => {
        const el = document.createElement("div");
        el.className = "warn " + x.lvl;
        el.innerHTML = `<span class="w-jp"></span><span class="w-txt"><span class="w-main"></span><span class="w-sub"></span></span>`;
        el.querySelector(".w-jp").textContent = x.jp;
        el.querySelector(".w-main").textContent = x.txt;
        el.querySelector(".w-sub").textContent = x.sub;
        return el;
      }),
    );
  }
  const crit = new Set(w.filter((x) => x.lvl === "crit").map((x) => x.txt));
  for (const c of crit) if (!prevCrit.has(c)) { beep(); break; }
  prevCrit = crit;
}

function renderLost() {
  // A short grace period so a single dropped event doesn't flash the screen.
  const down = S.everConnected ? !S.connected || performance.now() - S.lastMsg > 3000 : !S.connected && performance.now() > 2500;
  $("lost").classList.toggle("show", down);
  if (down) {
    verdict($("magi-link"), "fail");
  }
}

// ------------------------------------------------------------------ timer
const timer = prefs.get("timer", { running: false, start: 0, acc: 0 });
function timerElapsed() {
  return timer.acc + (timer.running ? (Date.now() - timer.start) / 1000 : 0);
}
function timerToggle() {
  if (timer.running) {
    timer.acc = timerElapsed();
    timer.running = false;
  } else {
    timer.start = Date.now();
    timer.running = true;
  }
  prefs.set("timer", timer);
}
function timerReset() {
  timer.running = false;
  timer.acc = 0;
  prefs.set("timer", timer);
}
function renderTimer() {
  const e = timerElapsed();
  const m = Math.floor(e / 60);
  const s = e - m * 60;
  $("timer-v").textContent = `${String(m).padStart(2, "0")}:${s.toFixed(1).padStart(4, "0")}`;
  $("timer").classList.toggle("running", timer.running);
  $("timer-state").textContent = timer.running ? "RUNNING" : e > 0 ? "HOLD" : "STANDBY";
  $("clock").textContent = new Date().toTimeString().slice(0, 8);
}
$("timer").addEventListener("click", (e) => (e.shiftKey ? timerReset() : timerToggle()));
$("timer").addEventListener("dblclick", timerReset);

// ------------------------------------------------------------ interaction
const drawer = $("drawer");
function toggleDrawer(force) {
  drawer.classList.toggle("open", force ?? !drawer.classList.contains("open"));
  if (drawer.classList.contains("open")) {
    renderLinks();
    renderLog();
  }
}
$("magi").addEventListener("click", () => toggleDrawer());
$("drawer-close").addEventListener("click", () => toggleDrawer(false));
const help = $("help");
help.addEventListener("click", () => help.classList.remove("show"));

// Deliberately no W/A/S/D or arrow keys: basestation main.py reads those
// globally (pynput) to drive, so they must not double as HUD shortcuts.
window.addEventListener("keydown", (e) => {
  if (e.ctrlKey || e.altKey || e.metaKey) return;
  const k = e.key.toLowerCase();
  const handled = {
    " ": () => cams.swap(),
    c: () => cams.swap(),
    v: () => cams.toggleAuto(),
    g: () => cams.toggleGuides(),
    f: () => map.toggleFollow(),
    r: () => map.rotate(),
    0: () => map.resetView(),
    l: () => toggleDrawer(),
    t: () => (e.shiftKey ? timerReset() : timerToggle()),
    m: () => {
      muted = !muted;
      prefs.set("muted", muted);
      showTicker({ lvl: 30, node: "hud", msg: muted ? "warning tones muted (M)" : "warning tones on (M)" });
    },
    h: () => help.classList.toggle("show"),
    "?": () => help.classList.toggle("show"),
    escape: () => {
      help.classList.remove("show");
      toggleDrawer(false);
    },
  }[k];
  if (handled) {
    e.preventDefault();
    handled();
  }
});

// ------------------------------------------------------------- main loop
// Each widget draws in isolation: a bug in one must never freeze the others,
// least of all the camera.
const painters = [() => cams.frame(), () => map.frame(), ...surfaces.map((s) => () => s.frame())];
const painterErrors = new WeakSet();
function frame() {
  for (const paint of painters) {
    try {
      paint();
    } catch (e) {
      if (!painterErrors.has(paint)) console.error("HUD widget failed to draw", e);
      painterErrors.add(paint);
    }
  }
  requestAnimationFrame(frame);
}
window.__hud = S; // for poking at state from the devtools console

setInterval(() => {
  renderHeader();
  renderWarnings();
  renderLost();
  renderTimer();
  if (drawer.classList.contains("open")) renderLinks();
}, 100);

// ------------------------------------------------------------------- boot
function boot() {
  const lines = [
    "MAGI-01 CASPER ........ LINK MONITOR",
    "MAGI-02 BALTHASAR ..... DRIVE MONITOR",
    "MAGI-03 MELCHIOR ...... LOCALISATION",
    "MONITOR MODE · NO COMMAND OUTPUT",
  ];
  const el = $("boot-lines");
  lines.forEach((l, i) =>
    setTimeout(() => {
      el.textContent += (i ? "\n" : "") + l + "  OK";
      el.style.whiteSpace = "pre";
    }, 120 + i * 150),
  );
  setTimeout(() => $("boot").classList.add("done"), 120 + lines.length * 150 + 350);
  $("boot").addEventListener("click", () => $("boot").classList.add("done"));
}

boot();
connect();
requestAnimationFrame(frame);
