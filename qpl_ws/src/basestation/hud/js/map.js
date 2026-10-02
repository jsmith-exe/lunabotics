// Tactical map: top-down arena with zones, global costmap, Nav2 plan, rover
// footprint, trail and the berm bearing. World units are metres in the map
// frame (x right, y up before rotation).

import { C, FONT_LABEL, FONT_MONO, Surface, clamp, deg, isNum, glow, noGlow, label, boxLabel, cross, prefs } from "./util.js";

const ZONE_STYLE = {
  EXCAVATION: { color: "#ff6a2a", fill: "rgba(255,106,42,0.07)", label: "EXCAVATION" },
  START: { color: "#3dff7a", fill: "rgba(61,255,122,0.10)", label: "START" },
  CONSTRUCTION: { color: "#48f0d8", fill: "rgba(72,240,216,0.08)", label: "CONSTRUCTION" },
  BERM: { color: "#ffc23a", fill: "rgba(255,194,58,0.18)", label: "BERM" },
};
const DRAW_ORDER = ["EXCAVATION", "START", "CONSTRUCTION", "BERM"];
const TRAIL_MAX = 1500;
const TRAIL_STEP = 0.03; // m between stored trail points
const FOOT_H = 30;       // px reserved for the readout bar

export class TacticalMap {
  constructor(canvas, S) {
    this.S = S;
    this.canvas = canvas;
    this.rot = prefs.get("mapRot", 0);
    this.follow = prefs.get("mapFollow", false);
    this.zoom = 1;
    this.center = null; // world point at the view centre; null = arena centre
    this.trail = [];
    this.trailFrame = null;
    this.costImg = null;
    this.cost = null;
    this.hatch = {};
    this.surface = new Surface(canvas, (ctx, w, h) => this.draw(ctx, w, h));
    this.bindInput();
    this.syncButtons();
  }

  // ---------------------------------------------------------------- data in
  onConfig() {
    this.resetView(false);
  }

  onCostmap(c) {
    if (!c.png) return;
    const img = new Image();
    img.onload = () => {
      this.costImg = img;
      this.cost = c;
    };
    img.src = "data:image/png;base64," + c.png;
  }

  onTele() {
    const d = this.S.derived;
    if (!d.pose) return;
    if (d.frame !== this.trailFrame) {
      this.trail = [];
      this.trailFrame = d.frame;
    }
    const last = this.trail[this.trail.length - 1];
    const p = [d.pose.x, d.pose.y];
    if (!last || Math.hypot(p[0] - last[0], p[1] - last[1]) > TRAIL_STEP) {
      this.trail.push(p);
      if (this.trail.length > TRAIL_MAX) this.trail.shift();
    }
  }

  // ------------------------------------------------------------- queries
  get arena() {
    return this.S.config?.arena ?? null;
  }

  /** Smallest arena zone containing the rover centre, for the header. */
  zoneName() {
    const d = this.S.derived;
    const a = this.arena;
    if (d.poseStale) return null;
    if (!a || d.frame !== "map" || !d.pose) return d.frame === "odom" ? "NO MAP FIX" : null;
    const { x, y } = d.pose;
    let best = null;
    for (const z of a.zones) {
      if (x >= z.x_min && x <= z.x_max && y >= z.y_min && y <= z.y_max) {
        const area = (z.x_max - z.x_min) * (z.y_max - z.y_min);
        if (!best || area < best.area) best = { name: z.name, area };
      }
    }
    if (best) return best.name;
    return x >= 0 && x <= a.width && y >= 0 && y <= a.length ? "TRANSIT" : "OUTSIDE";
  }

  footprint(pose) {
    const r = this.S.config?.rover ?? { length: 1.17, width: 0.74 };
    const hl = r.length / 2, hw = r.width / 2;
    const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    return [[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]].map(([px, py]) => [pose.x + c * px - s * py, pose.y + s * px + c * py]);
  }

  /** Smallest footprint-corner distance to the arena wall; negative = outside. */
  wallClearance() {
    const d = this.S.derived;
    const a = this.arena;
    if (!a || d.frame !== "map" || !d.pose || d.poseStale || !isNum(d.pose.yaw)) return null;
    let m = Infinity;
    for (const [x, y] of this.footprint(d.pose)) m = Math.min(m, x, a.width - x, y, a.length - y);
    return m;
  }

  berm() {
    const z = this.arena?.zones?.find((q) => q.name === "BERM");
    return z ? [(z.x_min + z.x_max) / 2, (z.y_min + z.y_max) / 2] : null;
  }

  // ------------------------------------------------------------ view state
  syncButtons() {
    document.getElementById("map-follow").classList.toggle("on", this.follow);
    document.getElementById("map-rotate").textContent = `ROT ${this.rot * 90}°`;
  }

  toggleFollow() {
    this.follow = !this.follow;
    prefs.set("mapFollow", this.follow);
    if (this.follow && this.zoom < 1.6) this.zoom = 2;
    if (!this.follow) this.center = null;
    this.syncButtons();
  }

  rotate() {
    this.rot = (this.rot + 1) % 4;
    prefs.set("mapRot", this.rot);
    this.syncButtons();
  }

  resetView(clearFollow = true) {
    this.zoom = 1;
    this.center = null;
    if (clearFollow && this.follow) {
      this.follow = false;
      prefs.set("mapFollow", false);
    }
    this.syncButtons();
  }

  bindInput() {
    const cv = this.canvas;
    cv.addEventListener("wheel", (e) => {
      e.preventDefault();
      const before = this.toWorld(e.offsetX, e.offsetY);
      this.zoom = clamp(this.zoom * Math.exp(-e.deltaY * 0.0015), 0.5, 12);
      if (!this.follow) {
        this.computeTransform(this.surface.w, this.surface.h);
        const after = this.toWorld(e.offsetX, e.offsetY);
        const c = this.viewCenter();
        this.center = [c[0] + before[0] - after[0], c[1] + before[1] - after[1]];
      }
    }, { passive: false });
    let drag = null;
    cv.addEventListener("pointerdown", (e) => {
      drag = { x: e.offsetX, y: e.offsetY, c: this.viewCenter() };
      cv.setPointerCapture(e.pointerId);
      cv.classList.add("drag");
    });
    cv.addEventListener("pointermove", (e) => {
      if (!drag) return;
      const dx = e.offsetX - drag.x, dy = e.offsetY - drag.y;
      if (Math.hypot(dx, dy) < 3) return;
      if (this.follow) {
        this.follow = false;
        prefs.set("mapFollow", false);
        this.syncButtons();
      }
      // Inverse of the linear part of the transform.
      const { a, b, c, d } = this.T;
      const det = a * d - b * c;
      this.center = [drag.c[0] - (d * dx - c * dy) / det, drag.c[1] - (-b * dx + a * dy) / det];
    });
    const end = () => {
      drag = null;
      cv.classList.remove("drag");
    };
    cv.addEventListener("pointerup", end);
    cv.addEventListener("pointercancel", end);
    cv.addEventListener("dblclick", () => this.resetView());
    document.getElementById("map-follow").addEventListener("click", () => this.toggleFollow());
    document.getElementById("map-rotate").addEventListener("click", () => this.rotate());
    document.getElementById("map-reset").addEventListener("click", () => this.resetView());
  }

  viewCenter() {
    const d = this.S.derived;
    if (this.follow && d.pose) return [d.pose.x, d.pose.y];
    if (this.center) return this.center;
    const a = this.arena;
    if (a && d.frame !== "odom") return [a.width / 2, a.length / 2];
    return d.pose ? [d.pose.x, d.pose.y] : [0, 0];
  }

  computeTransform(w, h) {
    const a = this.arena;
    const d = this.S.derived;
    const extent = a && d.frame !== "odom" ? [a.width + 1.2, a.length + 1.2] : [8, 8];
    const swap = this.rot % 2 === 1;
    const ew = swap ? extent[1] : extent[0];
    const eh = swap ? extent[0] : extent[1];
    const top = 40; // room for the title tag
    const availH = h - FOOT_H - top;
    const s = Math.min(w / ew, availH / eh) * this.zoom;
    const th = (this.rot * Math.PI) / 2;
    const [cx, cy] = this.viewCenter();
    const sx = w / 2, sy = top + availH / 2;
    const A = s * Math.cos(th), B = -s * Math.sin(th), Cc = -s * Math.sin(th), D = -s * Math.cos(th);
    this.T = { a: A, b: B, c: Cc, d: D, e: sx - A * cx - Cc * cy, f: sy - B * cx - D * cy, s };
  }

  toScreen(x, y) {
    const T = this.T;
    return [T.a * x + T.c * y + T.e, T.b * x + T.d * y + T.f];
  }

  toWorld(px, py) {
    const T = this.T;
    const det = T.a * T.d - T.b * T.c;
    const x = px - T.e, y = py - T.f;
    return [(T.d * x - T.c * y) / det, (-T.b * x + T.a * y) / det];
  }

  hatchFor(color) {
    if (this.hatch[color]) return this.hatch[color];
    const c = document.createElement("canvas");
    c.width = c.height = 10;
    const g = c.getContext("2d");
    g.strokeStyle = color;
    g.globalAlpha = 0.28;
    g.lineWidth = 1;
    g.beginPath();
    g.moveTo(0, 10);
    g.lineTo(10, 0);
    g.stroke();
    return (this.hatch[color] = g.createPattern(c, "repeat"));
  }

  frame() {
    this.surface.frame();
    this.updateFoot();
  }

  // ----------------------------------------------------------------- draw
  draw(ctx, w, h) {
    this.computeTransform(w, h);
    const S = this.S;
    const d = S.derived;
    const a = this.arena;
    const inMap = d.frame === "map";

    this.drawGrid(ctx, w, h);
    if (a) this.drawArena(ctx, a, inMap || !d.frame);
    if (this.costImg && this.cost && this.cost.frame === (d.frame || "map")) this.drawCostmap(ctx);
    if (S.plan && S.plan.frame === d.frame) this.drawPlan(ctx, S.plan.pts);
    this.drawTrail(ctx);
    if (inMap && d.pose) this.drawBermBearing(ctx, d.pose);
    if (S.tele?.tag && S.tele.tag.age < 4 && inMap) this.drawTagFix(ctx, S.tele.tag);
    if (d.pose && isNum(d.pose.yaw)) this.drawRover(ctx, d.pose);
    this.drawScale(ctx, w, h);

    if (!d.pose) {
      boxLabel(ctx, S.tele ? "NO ROVER POSE · WAITING FOR TF" : "NO TELEMETRY", w / 2, h / 2, { size: 13, color: C.amber });
    } else if (d.frame === "odom") {
      boxLabel(ctx, "ODOM FRAME · NOT ALIGNED TO ARENA", w / 2, 52, { size: 11, color: C.amber });
    }
  }

  drawGrid(ctx, w, h) {
    // 1 m graticule: faint lines, green crosses at the intersections.
    const corners = [this.toWorld(0, 0), this.toWorld(w, 0), this.toWorld(0, h), this.toWorld(w, h)];
    const xs = corners.map((p) => p[0]), ys = corners.map((p) => p[1]);
    const x0 = Math.floor(Math.min(...xs)), x1 = Math.ceil(Math.max(...xs));
    const y0 = Math.floor(Math.min(...ys)), y1 = Math.ceil(Math.max(...ys));
    if ((x1 - x0) * (y1 - y0) > 6000) return;
    ctx.strokeStyle = C.greenFaint;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = x0; x <= x1; x++) {
      ctx.moveTo(...this.toScreen(x, y0));
      ctx.lineTo(...this.toScreen(x, y1));
    }
    for (let y = y0; y <= y1; y++) {
      ctx.moveTo(...this.toScreen(x0, y));
      ctx.lineTo(...this.toScreen(x1, y));
    }
    ctx.stroke();
    ctx.strokeStyle = C.greenDim;
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    for (let x = x0; x <= x1; x++) for (let y = y0; y <= y1; y++) cross(ctx, ...this.toScreen(x, y), 3.5);
    ctx.stroke();
  }

  poly(ctx, pts) {
    ctx.beginPath();
    pts.forEach((p, i) => (i ? ctx.lineTo(...this.toScreen(...p)) : ctx.moveTo(...this.toScreen(...p))));
    ctx.closePath();
  }

  rect(z) {
    return [[z.x_min, z.y_min], [z.x_max, z.y_min], [z.x_max, z.y_max], [z.x_min, z.y_max]];
  }

  drawArena(ctx, a, aligned) {
    ctx.save();
    if (!aligned) ctx.globalAlpha = 0.35;
    const order = [...a.zones].sort((p, q) => DRAW_ORDER.indexOf(p.name) - DRAW_ORDER.indexOf(q.name));
    for (const z of order) {
      const st = ZONE_STYLE[z.name] ?? { color: C.orange, fill: "rgba(255,138,30,0.06)", label: z.name };
      this.poly(ctx, this.rect(z));
      ctx.fillStyle = st.fill;
      ctx.fill();
      ctx.fillStyle = this.hatchFor(st.color);
      ctx.fill();
      ctx.strokeStyle = st.color;
      ctx.lineWidth = z.name === "BERM" ? 2 : 1.2;
      ctx.setLineDash(z.name === "EXCAVATION" ? [6, 4] : []);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    // Arena wall + the safety buffer inside it.
    this.poly(ctx, [[0, 0], [a.width, 0], [a.width, a.length], [0, a.length]]);
    ctx.strokeStyle = C.orange;
    ctx.lineWidth = 2.5;
    glow(ctx, C.orange, 8);
    ctx.stroke();
    noGlow(ctx);
    if (a.buffer > 0) {
      const b = a.buffer;
      this.poly(ctx, [[b, b], [a.width - b, b], [a.width - b, a.length - b], [b, a.length - b]]);
      ctx.strokeStyle = C.orangeDim;
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 5]);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    // Edge ticks every metre, NERV-plot style (+01, +02 ...).
    for (let x = 0; x <= a.width + 1e-6; x += 1) {
      const [sx, sy] = this.toScreen(x, 0);
      label(ctx, `+${String(Math.round(x)).padStart(2, "0")}`, sx, sy + 13, { size: 9, color: C.greenDim, align: "center", font: FONT_MONO, weight: 400 });
    }
    for (let y = 1; y <= a.length + 1e-6; y += 1) {
      const [sx, sy] = this.toScreen(0, y);
      label(ctx, `+${String(Math.round(y)).padStart(2, "0")}`, sx - 5, sy + 3, { size: 9, color: C.greenDim, align: "right", font: FONT_MONO, weight: 400 });
    }
    // Zone labels on top of everything else in the arena layer.
    for (const z of order) {
      const st = ZONE_STYLE[z.name] ?? { color: C.orange, label: z.name };
      const cx = (z.x_min + z.x_max) / 2;
      // EXCAVATION spans the full width; put its label near the wall so the
      // START box inside it stays readable.
      // CONSTRUCTION contains the BERM; label it near its far edge.
      const cy = z.name === "EXCAVATION" ? z.y_min + (z.y_max - z.y_min) * 0.82
        : z.name === "CONSTRUCTION" ? z.y_max - 0.3 : (z.y_min + z.y_max) / 2;
      const [sx, sy] = this.toScreen(z.name === "EXCAVATION" ? z.x_min + 1.2 : cx, cy);
      boxLabel(ctx, st.label, sx, sy, { size: 10, color: st.color });
    }
    if (a.tag) {
      const [sx, sy] = this.toScreen(a.tag[0], a.tag[1]);
      ctx.fillStyle = C.purple;
      ctx.beginPath();
      ctx.moveTo(sx, sy - 6);
      ctx.lineTo(sx + 6, sy);
      ctx.lineTo(sx, sy + 6);
      ctx.lineTo(sx - 6, sy);
      ctx.closePath();
      ctx.fill();
      label(ctx, "TAG", sx + 9, sy + 4, { size: 9, color: C.purple });
    }
    ctx.restore();
  }

  drawCostmap(ctx) {
    const c = this.cost;
    const T = this.T;
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(T.a * dpr, T.b * dpr, T.c * dpr, T.d * dpr, T.e * dpr, T.f * dpr);
    ctx.translate(c.ox, c.oy);
    ctx.rotate(c.oyaw || 0);
    ctx.translate(0, c.h * c.res);
    ctx.scale(c.res, -c.res);
    ctx.imageSmoothingEnabled = true; // 5 cm cells read as blocks otherwise
    ctx.globalAlpha = 0.85;
    ctx.drawImage(this.costImg, 0, 0);
    ctx.restore();
  }

  drawPlan(ctx, pts) {
    if (!pts || pts.length < 2) return;
    ctx.strokeStyle = C.cyan;
    ctx.lineWidth = 2;
    ctx.setLineDash([7, 5]);
    glow(ctx, C.cyan, 6);
    ctx.beginPath();
    pts.forEach((p, i) => (i ? ctx.lineTo(...this.toScreen(...p)) : ctx.moveTo(...this.toScreen(...p))));
    ctx.stroke();
    ctx.setLineDash([]);
    noGlow(ctx);
    const [ex, ey] = this.toScreen(...pts[pts.length - 1]);
    ctx.strokeStyle = C.cyan;
    ctx.beginPath();
    ctx.arc(ex, ey, 7, 0, Math.PI * 2);
    cross(ctx, ex, ey, 11);
    ctx.stroke();
    label(ctx, "GOAL", ex + 10, ey - 8, { size: 10, color: C.cyan });
  }

  drawTrail(ctx) {
    const n = this.trail.length;
    if (n < 2) return;
    ctx.lineWidth = 2;
    // Fade from old to new in a few batches (cheaper than per-segment alpha).
    const batches = 6;
    for (let b = 0; b < batches; b++) {
      const i0 = Math.floor((n * b) / batches), i1 = Math.min(n - 1, Math.floor((n * (b + 1)) / batches));
      if (i1 <= i0) continue;
      ctx.strokeStyle = `rgba(255,138,30,${0.12 + (0.6 * (b + 1)) / batches})`;
      ctx.beginPath();
      ctx.moveTo(...this.toScreen(...this.trail[i0]));
      for (let i = i0 + 1; i <= i1; i++) ctx.lineTo(...this.toScreen(...this.trail[i]));
      ctx.stroke();
    }
  }

  drawBermBearing(ctx, pose) {
    const b = this.berm();
    if (!b) return;
    ctx.strokeStyle = "rgba(255,194,58,0.55)";
    ctx.lineWidth = 1.2;
    ctx.setLineDash([2, 6]);
    ctx.beginPath();
    ctx.moveTo(...this.toScreen(pose.x, pose.y));
    ctx.lineTo(...this.toScreen(...b));
    ctx.stroke();
    ctx.setLineDash([]);
  }

  drawTagFix(ctx, tag) {
    if (!isNum(tag.x)) return;
    const [sx, sy] = this.toScreen(tag.x, tag.y);
    const a = clamp(1 - tag.age / 4, 0.15, 1);
    ctx.strokeStyle = `rgba(155,107,255,${a})`;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(sx, sy, 9, 0, Math.PI * 2);
    cross(ctx, sx, sy, 14);
    ctx.stroke();
    label(ctx, `TAG FIX ${tag.age.toFixed(1)}s`, sx + 12, sy + 16, { size: 9, color: `rgba(155,107,255,${a})` });
  }

  drawRover(ctx, pose) {
    const r = this.S.config?.rover ?? { length: 1.17, width: 0.74, wheel_radius: 0.15, wheel_width: 0.15 };
    const fp = this.footprint(pose);
    const stale = this.S.derived.poseStale;
    const stalled = this.S.derived.stalled?.length > 0;
    const col = stale ? C.grey : stalled ? C.red : C.orange;
    // Body.
    this.poly(ctx, fp);
    ctx.fillStyle = stale ? "rgba(109,90,72,0.15)" : "rgba(255,138,30,0.16)";
    ctx.fill();
    ctx.strokeStyle = col;
    ctx.lineWidth = 2;
    if (stale) ctx.setLineDash([5, 4]);
    glow(ctx, col, stale ? 0 : 10);
    ctx.stroke();
    ctx.setLineDash([]);
    if (stale) {
      noGlow(ctx);
      const [sx, sy] = this.toScreen(pose.x, pose.y);
      boxLabel(ctx, `LAST KNOWN · ${pose.age < 600 ? pose.age.toFixed(0) + " S AGO" : "LONG AGO"}`, sx, sy - 34, { size: 10, color: C.grey });
      return;
    }
    // Front edge, bright, so the heading is never ambiguous.
    ctx.strokeStyle = "#fff3e0";
    ctx.lineWidth = 3.5;
    ctx.beginPath();
    ctx.moveTo(...this.toScreen(...fp[0]));
    ctx.lineTo(...this.toScreen(...fp[1]));
    ctx.stroke();
    noGlow(ctx);
    // Wheels.
    const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    const wx = r.length / 2 - r.wheel_radius, wy = r.width / 2;
    for (const [px, py] of [[wx, wy], [wx, -wy], [-wx, wy], [-wx, -wy]]) {
      const hl = r.wheel_radius, hw = r.wheel_width / 2;
      const pts = [[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]].map(([qx, qy]) => {
        const lx = px + qx, ly = py + qy;
        return [pose.x + c * lx - s * ly, pose.y + s * lx + c * ly];
      });
      this.poly(ctx, pts);
      ctx.fillStyle = "rgba(0,0,0,0.85)";
      ctx.fill();
      ctx.strokeStyle = col;
      ctx.lineWidth = 1;
      ctx.stroke();
    }
    // Heading chevron beyond the nose.
    const tip = r.length / 2 + 0.45;
    const chev = [[tip, 0], [tip - 0.22, 0.16], [tip - 0.14, 0], [tip - 0.22, -0.16]].map(([px, py]) => [pose.x + c * px - s * py, pose.y + s * px + c * py]);
    this.poly(ctx, chev);
    ctx.fillStyle = C.amber;
    glow(ctx, C.amber, 8);
    ctx.fill();
    noGlow(ctx);
    // Command vector: where the operator is pushing it right now.
    const d = this.S.derived;
    if (d.cmd && Math.abs(d.cmdVx) > 0.02) {
      const len = clamp(d.cmdVx, -1, 1) * 1.4;
      const base = this.toScreen(pose.x, pose.y);
      const end = this.toScreen(pose.x + c * len, pose.y + s * len);
      ctx.strokeStyle = C.green;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(...base);
      ctx.lineTo(...end);
      ctx.stroke();
    }
  }

  drawScale(ctx, w, h) {
    // Pick a round length that is 40-120 px long.
    const s = this.T.s;
    const opts = [0.25, 0.5, 1, 2, 5];
    const m = opts.find((o) => o * s >= 40) ?? 5;
    const x = 14, y = h - FOOT_H - 12;
    ctx.strokeStyle = C.text;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x, y - 4);
    ctx.lineTo(x, y);
    ctx.lineTo(x + m * s, y);
    ctx.lineTo(x + m * s, y - 4);
    ctx.stroke();
    label(ctx, `${m} M`, x + (m * s) / 2, y - 6, { size: 10, color: C.text, align: "center" });
    // Axis key: +X / +Y arrows after rotation.
    const ox = w - 40, oy = h - FOOT_H - 34;
    const [ax, ay] = [this.T.a / this.T.s, this.T.b / this.T.s];
    const [bx, by] = [this.T.c / this.T.s, this.T.d / this.T.s];
    ctx.strokeStyle = C.textDim;
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(ox, oy);
    ctx.lineTo(ox + ax * 20, oy + ay * 20);
    ctx.moveTo(ox, oy);
    ctx.lineTo(ox + bx * 20, oy + by * 20);
    ctx.stroke();
    label(ctx, "X", ox + ax * 28, oy + ay * 28 + 4, { size: 10, color: C.textDim, align: "center" });
    label(ctx, "Y", ox + bx * 28, oy + by * 28 + 4, { size: 10, color: C.textDim, align: "center" });
  }

  updateFoot() {
    const d = this.S.derived;
    const set = (id, v) => {
      const el = document.getElementById(id);
      if (el.textContent !== v) el.textContent = v;
    };
    const p = d.pose;
    set("m-x", p && isNum(p.x) ? p.x.toFixed(2) : "—");
    set("m-y", p && isNum(p.y) ? p.y.toFixed(2) : "—");
    set("m-h", p && isNum(p.yaw) ? `${((deg(p.yaw) % 360) + 360) % 360 | 0}°` : "—");
    const b = this.berm();
    if (p && b && d.frame === "map") {
      const dist = Math.hypot(b[0] - p.x, b[1] - p.y);
      let rel = deg(Math.atan2(b[1] - p.y, b[0] - p.x) - p.yaw);
      rel = ((rel + 540) % 360) - 180;
      const side = Math.abs(rel) < 3 ? "AHEAD" : rel > 0 ? `${Math.abs(rel).toFixed(0)}° L` : `${Math.abs(rel).toFixed(0)}° R`;
      set("m-berm", `${dist.toFixed(2)} m · ${side}`);
    } else set("m-berm", "—");
    const ft = document.getElementById("m-frame");
    const f = d.frame === "map" ? "MAP FRAME" : d.frame === "odom" ? "ODOM ONLY" : "NO POSE";
    set("m-frame", f);
    ft.className = "frame-tag " + (d.frame === "map" ? "ok" : "warn");
  }
}
