// Bottom-strip instruments. Each draw* gets CSS-pixel (w, h) and the shared state.

import { C, FONT_MONO, clamp, deg, isNum, fmtSigned, glow, noGlow, label, cross, TILT_CAUTION, TILT_CRIT } from "./util.js";

const TOP = 4; // canvases already sit below the panel tag

function setText(id, v) {
  const el = document.getElementById(id);
  if (el && el.textContent !== v) el.textContent = v;
}

// ------------------------------------------------------------------ DRIVE
/** Command vs measured motion on one stick-style plane: up = forward,
 * left = turning left (+ω). Green dot = command, orange ring = measured. */
export function drawDrive(ctx, w, h, S) {
  const d = S.derived;
  const rover = S.config?.rover ?? { max_linear: 1, max_angular: 1 };
  const size = Math.max(40, Math.min(w - 26, h - 18));
  const cx = 14 + size / 2 + (w - 26 - size) / 2, cy = TOP + 4 + size / 2;
  const half = size / 2;

  ctx.strokeStyle = C.orangeDim;
  ctx.lineWidth = 1;
  ctx.strokeRect(cx - half, cy - half, size, size);
  ctx.strokeStyle = C.greenFaint;
  ctx.beginPath();
  for (const f of [-0.5, 0.5]) {
    ctx.moveTo(cx + f * half, cy - half);
    ctx.lineTo(cx + f * half, cy + half);
    ctx.moveTo(cx - half, cy + f * half);
    ctx.lineTo(cx + half, cy + f * half);
  }
  ctx.stroke();
  ctx.strokeStyle = C.greenDim;
  ctx.beginPath();
  for (const fx of [-0.5, 0, 0.5]) for (const fy of [-0.5, 0, 0.5]) cross(ctx, cx + fx * half, cy + fy * half, 4);
  ctx.stroke();
  ctx.strokeStyle = C.orangeFaint;
  ctx.beginPath();
  ctx.moveTo(cx - half, cy);
  ctx.lineTo(cx + half, cy);
  ctx.moveTo(cx, cy - half);
  ctx.lineTo(cx, cy + half);
  ctx.stroke();
  label(ctx, "FWD", cx, cy - half + 11, { size: 9, color: C.textDim, align: "center" });
  label(ctx, "REV", cx, cy + half - 4, { size: 9, color: C.textDim, align: "center" });
  label(ctx, "L", cx - half + 4, cy - 4, { size: 9, color: C.textDim });
  label(ctx, "R", cx + half - 4, cy - 4, { size: 9, color: C.textDim, align: "right" });

  const px = (wz) => cx - clamp(wz / rover.max_angular, -1, 1) * half;
  const py = (vx) => cy - clamp(vx / rover.max_linear, -1, 1) * half;

  if (isNum(d.vx) && isNum(d.wz)) {
    ctx.strokeStyle = C.orange;
    ctx.lineWidth = 2;
    glow(ctx, C.orange, 8);
    ctx.beginPath();
    ctx.arc(px(d.wz), py(d.vx), 7, 0, Math.PI * 2);
    ctx.stroke();
    noGlow(ctx);
  }
  if (d.cmd) {
    const x = px(d.cmdWz), y = py(d.cmdVx);
    ctx.strokeStyle = "rgba(61,255,122,0.5)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(x, y);
    ctx.stroke();
    ctx.fillStyle = C.green;
    glow(ctx, C.green, 10);
    ctx.beginPath();
    ctx.arc(x, y, 4.5, 0, Math.PI * 2);
    ctx.fill();
    noGlow(ctx);
  }

  setText("r-v", isNum(d.vx) ? fmtSigned(d.vx, 2) : "—");
  setText("r-w", isNum(d.wz) ? fmtSigned(d.wz, 2) : "—");
  const src = S.tele?.source;
  setText("r-cmd", d.cmd ? `${fmtSigned(d.cmdVx, 2)}  ${fmtSigned(d.cmdWz, 2)}` : "NONE");
}

// ----------------------------------------------------------------- WHEELS
export function drawWheels(ctx, w, h, S) {
  const t = S.tele;
  const r = S.config?.rover?.wheel_radius ?? 0.15;
  const maxV = S.config?.rover?.max_linear ?? 1;
  const stalled = new Set(S.derived.stalled || []);
  const fresh = S.derived.jointsFresh;

  const bodyW = Math.min(w * 0.2, 64), bodyH = Math.min(h - 30, 150);
  const cx = w / 2, cy = TOP + 8 + bodyH / 2 + 4;
  ctx.strokeStyle = C.orangeDim;
  ctx.lineWidth = 1.2;
  ctx.strokeRect(cx - bodyW / 2, cy - bodyH / 2, bodyW, bodyH);
  // Nose marker.
  ctx.fillStyle = C.amber;
  ctx.beginPath();
  ctx.moveTo(cx, cy - bodyH / 2 - 8);
  ctx.lineTo(cx - 6, cy - bodyH / 2 - 1);
  ctx.lineTo(cx + 6, cy - bodyH / 2 - 1);
  ctx.closePath();
  ctx.fill();

  const barW = Math.max(10, Math.min(18, w * 0.07));
  const barH = bodyH * 0.42;
  const gapX = bodyW / 2 + barW / 2 + 6;
  const spots = { fl: [-gapX, -bodyH / 4], fr: [gapX, -bodyH / 4], rl: [-gapX, bodyH / 4], rr: [gapX, bodyH / 4] };
  for (const [k, [ox, oy]] of Object.entries(spots)) {
    const x = cx + ox, y = cy + oy;
    const omega = t?.wheels?.[k];
    const v = isNum(omega) ? omega * r : null;
    const st = stalled.has(k);
    ctx.strokeStyle = st ? C.red : C.orange;
    ctx.lineWidth = st ? 2 : 1.2;
    if (st) glow(ctx, C.red, 10);
    ctx.strokeRect(x - barW / 2, y - barH / 2, barW, barH);
    noGlow(ctx);
    // Centre line, then fill up (fwd) or down (rev).
    ctx.strokeStyle = C.orangeDim;
    ctx.beginPath();
    ctx.moveTo(x - barW / 2, y);
    ctx.lineTo(x + barW / 2, y);
    ctx.stroke();
    if (v !== null && fresh) {
      const f = clamp(v / maxV, -1, 1);
      ctx.fillStyle = f >= 0 ? C.green : C.amber;
      glow(ctx, ctx.fillStyle, 6);
      const fh = (Math.abs(f) * barH) / 2;
      ctx.fillRect(x - barW / 2 + 2, f >= 0 ? y - fh : y, barW - 4, fh);
      noGlow(ctx);
    }
    const left = ox < 0;
    const tx = left ? x - barW / 2 - 5 : x + barW / 2 + 5;
    label(ctx, k.toUpperCase(), tx, y - 4, { size: 10, color: st ? C.red : C.textDim, align: left ? "right" : "left" });
    label(ctx, v !== null && fresh ? fmtSigned(v, 2) : "—", tx, y + 10, { size: 11, color: st ? C.red : C.text, align: left ? "right" : "left", font: FONT_MONO, weight: 400 });
    if (st) label(ctx, "STALL", x, y + barH / 2 + 12, { size: 10, color: C.red, align: "center" });
  }
  label(ctx, "SURFACE SPEED M/S", cx, h - 6, { size: 9, color: C.textDim, align: "center" });
}

// --------------------------------------------------------------- ATTITUDE
function tiltColor(a) {
  const x = Math.abs(a);
  return x >= TILT_CRIT ? C.red : x >= TILT_CAUTION ? C.amber : C.green;
}

function dial(ctx, cx, cy, rad, angleDeg, title, sub, drawShape) {
  // Arc scale ±40°, centred on the top.
  const range = 40;
  const toA = (dgr) => -Math.PI / 2 + (dgr * Math.PI) / 180;
  const band = (from, to, col) => {
    ctx.strokeStyle = col;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(cx, cy, rad, toA(from), toA(to));
    ctx.stroke();
  };
  band(-TILT_CAUTION, TILT_CAUTION, "rgba(61,255,122,0.55)");
  band(TILT_CAUTION, TILT_CRIT, "rgba(255,194,58,0.7)");
  band(-TILT_CRIT, -TILT_CAUTION, "rgba(255,194,58,0.7)");
  band(TILT_CRIT, range, "rgba(255,45,31,0.8)");
  band(-range, -TILT_CRIT, "rgba(255,45,31,0.8)");
  ctx.strokeStyle = C.textDim;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let a = -range; a <= range; a += 10) {
    const an = toA(a);
    ctx.moveTo(cx + Math.cos(an) * (rad + 3), cy + Math.sin(an) * (rad + 3));
    ctx.lineTo(cx + Math.cos(an) * (rad + (a % 20 === 0 ? 9 : 6)), cy + Math.sin(an) * (rad + (a % 20 === 0 ? 9 : 6)));
  }
  ctx.stroke();

  const has = isNum(angleDeg);
  const a = has ? angleDeg : 0;
  const col = has ? tiltColor(a) : C.grey;
  // Needle.
  const an = toA(clamp(a, -range - 4, range + 4));
  ctx.strokeStyle = col;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(cx + Math.cos(an) * (rad - 8), cy + Math.sin(an) * (rad - 8));
  ctx.lineTo(cx + Math.cos(an) * (rad + 10), cy + Math.sin(an) * (rad + 10));
  ctx.stroke();
  // Level reference.
  ctx.strokeStyle = C.orangeFaint;
  ctx.beginPath();
  ctx.moveTo(cx - rad * 0.85, cy);
  ctx.lineTo(cx + rad * 0.85, cy);
  ctx.stroke();
  // Rotated silhouette.
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate((a * Math.PI) / 180);
  ctx.strokeStyle = col;
  ctx.fillStyle = "rgba(0,0,0,0.6)";
  ctx.lineWidth = 1.6;
  glow(ctx, col, 6);
  drawShape(ctx, rad * 0.62);
  noGlow(ctx);
  ctx.restore();

  label(ctx, title, cx, cy + rad * 0.62 + 14, { size: 10, color: C.textDim, align: "center" });
  glow(ctx, col, 6);
  label(ctx, has ? `${Math.abs(a).toFixed(1)}°` : "—", cx, cy + rad * 0.62 + 32, { size: 17, color: has ? col : C.grey, align: "center", font: FONT_MONO });
  noGlow(ctx);
  if (has && Math.abs(a) >= 0.5) label(ctx, sub, cx, cy + rad * 0.62 + 45, { size: 9, color: C.textDim, align: "center" });
}

export function drawAttitude(ctx, w, h, S) {
  const att = S.derived.att;
  const rad = Math.max(16, Math.min(w / 4 - 14, (h - 62) / 1.55));
  const cy = TOP + rad + 12;
  const pitch = att && isNum(att.pitch) ? deg(att.pitch) : null;
  const roll = att && isNum(att.roll) ? deg(att.roll) : null;
  // Side view (front to the right). ROS +pitch = nose down = clockwise here.
  dial(ctx, w * 0.27, cy, rad, pitch, "PITCH", pitch > 0 ? "NOSE DOWN" : "NOSE UP", (g, s) => {
    g.beginPath();
    g.rect(-s, -s * 0.32, s * 2, s * 0.38);
    g.fill();
    g.stroke();
    g.beginPath();
    g.moveTo(s * 0.55, -s * 0.32);
    g.lineTo(s * 0.35, -s * 0.62);
    g.lineTo(-s * 0.2, -s * 0.62);
    g.stroke();
    for (const x of [-s * 0.68, s * 0.68]) {
      g.beginPath();
      g.arc(x, s * 0.16, s * 0.26, 0, Math.PI * 2);
      g.fill();
      g.stroke();
    }
    g.beginPath();
    g.moveTo(s * 1.05, -s * 0.12);
    g.lineTo(s * 1.28, -s * 0.12);
    g.stroke();
  });
  // Rear view (rover's left on screen left). +roll = left side up = clockwise.
  dial(ctx, w * 0.73, cy, rad, roll, "ROLL", roll > 0 ? "RIGHT SIDE LOW" : "LEFT SIDE LOW", (g, s) => {
    g.beginPath();
    g.rect(-s * 0.7, -s * 0.5, s * 1.4, s * 0.52);
    g.fill();
    g.stroke();
    for (const x of [-s * 0.88, s * 0.88]) {
      g.beginPath();
      g.rect(x - s * 0.17, -s * 0.12, s * 0.34, s * 0.52);
      g.fill();
      g.stroke();
    }
  });
}

// -------------------------------------------------------------- IMPLEMENT
let drumPhase = 0;
let lastT = performance.now();

export function drawImplement(ctx, w, h, S) {
  const t = S.tele;
  const now = performance.now();
  const dt = Math.min(0.1, (now - lastT) / 1000);
  lastT = now;
  // Joint feedback that has gone stale is shown as missing, not frozen.
  const live = S.derived.jointsFresh;
  const drum = { ...(t?.drum ?? {}) };
  const lift = { ...(t?.lift ?? {}) };
  if (!live) {
    drum.vel = null;
    lift.pos = {};
  }
  const vel = isNum(drum.vel) ? drum.vel : 0;
  drumPhase += vel * dt;

  // Drum: spinning spokes driven by the measured joint velocity.
  const rad = Math.max(14, Math.min(w * 0.2, (h - 62) / 2));
  const cx = 14 + rad, cy = TOP + 10 + rad;
  ctx.strokeStyle = C.orange;
  ctx.lineWidth = 2;
  glow(ctx, C.orange, 6);
  ctx.beginPath();
  ctx.arc(cx, cy, rad, 0, Math.PI * 2);
  ctx.stroke();
  noGlow(ctx);
  ctx.strokeStyle = Math.abs(vel) > 0.05 ? C.green : C.orangeDim;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  for (let i = 0; i < 6; i++) {
    const a = drumPhase + (i * Math.PI) / 3;
    ctx.moveTo(cx + Math.cos(a) * rad * 0.25, cy + Math.sin(a) * rad * 0.25);
    ctx.lineTo(cx + Math.cos(a) * rad * 0.88, cy + Math.sin(a) * rad * 0.88);
  }
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(cx, cy, rad * 0.25, 0, Math.PI * 2);
  ctx.stroke();

  const spinCmd = (drum.teleop && drum.teleop.age < 0.5 ? drum.teleop : null) || (drum.auto && drum.auto.age < 0.5 ? drum.auto : null);
  const spinSrc = spinCmd === drum.teleop ? "TELEOP" : "AUTO";
  label(ctx, "DRUM", cx, cy + rad + 14, { size: 10, color: C.textDim, align: "center" });
  label(ctx, isNum(drum.vel) ? `${fmtSigned(drum.vel, 1)} r/s` : "—", cx, cy + rad + 28, { size: 12, color: C.text, align: "center", font: FONT_MONO, weight: 400 });
  label(ctx, spinCmd ? `CMD ${fmtSigned(spinCmd.v, 2)} ${spinSrc}` : "CMD —", cx, cy + rad + 41, { size: 9, color: spinCmd ? C.green : C.grey, align: "center" });

  // Lift: left/right actuator positions with the commanded target marker.
  // Laid out in whatever width is left right of the drum.
  const x0 = cx + rad + 18;
  const avail = w - x0 - 12;
  const barW = clamp(avail * 0.22, 8, 18);
  const gap = clamp(avail * 0.14, 8, 20);
  const by = TOP + 30;
  const barH = Math.max(20, h - by - 34);
  const target = lift.cmd && lift.cmd.age < 5 ? lift.cmd.v : null;
  const lpos = lift.pos ?? {};
  const cramped = barW + gap < 34;
  const short = (v) => v.toFixed(2).replace(/^0/, "");
  label(ctx, cramped && isNum(lpos.l) && isNum(lpos.r) ? `LIFT ${short(lpos.l)}/${short(lpos.r)}` : "LIFT", x0, TOP + 12, { size: cramped ? 9 : 10, color: C.textDim });
  label(ctx, isNum(target) ? `TGT ${target.toFixed(2)}` : "TGT —", x0, TOP + 24, { size: 9, color: isNum(target) ? C.green : C.grey });
  ["l", "r"].forEach((k, i) => {
    const x = x0 + 4 + i * (barW + gap);
    const pos = lift.pos?.[k];
    ctx.strokeStyle = C.orange;
    ctx.lineWidth = 1.2;
    ctx.strokeRect(x, by, barW, barH);
    if (isNum(pos)) {
      const f = clamp(pos, 0, 1);
      ctx.fillStyle = C.orange;
      glow(ctx, C.orange, 6);
      ctx.fillRect(x + 2, by + barH - f * barH, barW - 4, f * barH);
      noGlow(ctx);
    }
    if (isNum(target)) {
      const ty = by + barH - clamp(target, 0, 1) * barH;
      ctx.strokeStyle = C.green;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x - 4, ty);
      ctx.lineTo(x + barW + 4, ty);
      ctx.stroke();
    }
    label(ctx, k.toUpperCase(), x + barW / 2, by + barH + 12, { size: 10, color: C.textDim, align: "center" });
    if (barW + gap >= 34) label(ctx, isNum(pos) ? pos.toFixed(2) : "—", x + barW / 2, by + barH + 25, { size: 10, color: C.text, align: "center", font: FONT_MONO, weight: 400 });
  });
  const lp = lift.pos ?? {};
  if (isNum(lp.l) && isNum(lp.r) && Math.abs(lp.l - lp.r) > 0.08) {
    label(ctx, `SKEW ${Math.abs(lp.l - lp.r).toFixed(2)}`, x0 + 4 + 2 * (barW + gap) - gap + 6, by + 10, { size: 10, color: C.amber });
  }
}

// ------------------------------------------------------------------ TRACE
export class TraceHistory {
  constructor(seconds, hz) {
    this.seconds = seconds;
    this.max = seconds * hz + 20;
    this.pts = [];
  }
  push(tele, d) {
    this.pts.push({ t: tele.t, vx: d.vx, cmd: d.cmd ? d.cmdVx : 0, wz: d.wz });
    if (this.pts.length > this.max) this.pts.shift();
  }
}

/** Scrolling history, styled after the NERV psychograph plots: orange axes,
 * green graticule crosses, signed "+0.5" axis labels. */
export function drawTrace(ctx, w, h, S, hist) {
  const left = 40, right = 12, top = TOP + 10, bottom = 20;
  const pw = w - left - right, ph = h - top - bottom;
  if (pw < 20 || ph < 20) return;
  const yOf = (v) => top + ph / 2 - clamp(v, -1.1, 1.1) * (ph / 2.2);

  // Graticule.
  ctx.strokeStyle = C.greenDim;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let s = 0; s <= hist.seconds; s += 5) {
    const x = left + pw - (s / hist.seconds) * pw;
    ctx.moveTo(x, top);
    ctx.lineTo(x, top + ph);
  }
  ctx.stroke();
  ctx.strokeStyle = C.green;
  ctx.beginPath();
  // Crosses midway between the 5 s lines.
  for (let s = 2.5; s < hist.seconds; s += 5) {
    const x = left + pw - (s / hist.seconds) * pw;
    for (const v of [-1, -0.5, 0.5, 1]) cross(ctx, x, yOf(v), 3);
  }
  ctx.stroke();
  // Axes.
  ctx.strokeStyle = C.orange;
  ctx.lineWidth = 2;
  glow(ctx, C.orange, 6);
  ctx.beginPath();
  ctx.moveTo(left, top);
  ctx.lineTo(left, top + ph);
  ctx.moveTo(left, yOf(0));
  ctx.lineTo(left + pw, yOf(0));
  ctx.stroke();
  noGlow(ctx);
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (const v of [-1, -0.5, 0.5, 1]) {
    ctx.moveTo(left - 5, yOf(v));
    ctx.lineTo(left, yOf(v));
  }
  ctx.stroke();
  for (const v of [-1, -0.5, 0, 0.5, 1]) {
    label(ctx, v === 0 ? "0" : fmtSigned(v, 1), left - 8, yOf(v) + 4, { size: 10, color: C.orange, align: "right", font: FONT_MONO, weight: 400 });
  }
  for (let s = 0; s <= hist.seconds; s += 5) {
    const x = left + pw - (s / hist.seconds) * pw;
    label(ctx, s === 0 ? "NOW" : `−${s}`, x, top + ph + 14, { size: 9, color: C.textDim, align: "center", font: FONT_MONO, weight: 400 });
  }

  const pts = hist.pts;
  if (pts.length < 2) return;
  const tNow = pts[pts.length - 1].t;
  const xOf = (t) => left + pw - ((tNow - t) / hist.seconds) * pw;
  ctx.save();
  ctx.beginPath();
  ctx.rect(left, top - 2, pw, ph + 4);
  ctx.clip();
  const series = (key, col, width, step) => {
    ctx.strokeStyle = col;
    ctx.lineWidth = width;
    glow(ctx, col, width > 1.5 ? 8 : 0);
    ctx.beginPath();
    let pen = false;
    let prevY = null;
    for (const p of pts) {
      if (tNow - p.t > hist.seconds + 0.5) continue;
      const v = p[key];
      if (!isNum(v)) {
        pen = false;
        continue;
      }
      const x = xOf(p.t), y = yOf(v);
      if (!pen) ctx.moveTo(x, y);
      else if (step) {
        ctx.lineTo(x, prevY);
        ctx.lineTo(x, y);
      } else ctx.lineTo(x, y);
      pen = true;
      prevY = y;
    }
    ctx.stroke();
    noGlow(ctx);
  };
  series("wz", "rgba(72,240,216,0.75)", 1.2, false);
  series("cmd", C.green, 1.6, true);
  series("vx", C.orange, 2.4, false);
  ctx.restore();
}
