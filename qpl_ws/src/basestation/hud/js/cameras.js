// Camera deck: which feed is in the main pane vs the corner, the MJPEG <img>
// elements, NO SIGNAL handling and the main-pane overlay (reticle, heading
// tape, speed, drive guides projected through the real camera model).

import { C, FONT_LABEL, FONT_MONO, Surface, clamp, deg, isNum, fmtSigned, glow, noGlow, label, prefs } from "./util.js";

const KEYS = ["front", "rear"];
const NUM = { front: "01", rear: "02" };
// Guide colours by distance from the bumper, like a reversing camera.
const GUIDE_TICKS = [
  [0.5, C.red],
  [1.0, C.amber],
  [1.5, C.green],
  [2.0, C.green],
];
const GUIDE_LEN = 2.2; // m of predicted path drawn
const PIVOT_KAPPA = 1.2; // 1/m: turns tighter than this are shown as a pivot
// Auto-reverse view: switch after the command has been reversing this long.
const AUTO_HOLD_S = 0.35;

// Fallback model when camera_info/TF never arrive: a RealSense-ish 87° HFOV
// camera 0.5 m up at the bumper, pitched 20° down. Marked EST on screen.
function fallbackModel(key, rover) {
  const w = 1280, h = 800;
  const f = w / 2 / Math.tan((87 * Math.PI) / 360);
  const p = (20 * Math.PI) / 180;
  const sgn = key === "front" ? 1 : -1;
  // Optical frame: z forward, x right, y down. Columns of R are the optical
  // axes expressed in base_footprint.
  const zx = sgn * Math.cos(p), zz = -Math.sin(p); // forward, tilted down
  const xAxis = [0, -sgn, 0];                      // image right
  const zAxis = [zx, 0, zz];
  const yAxis = [ // y = z × x
    zAxis[1] * xAxis[2] - zAxis[2] * xAxis[1],
    zAxis[2] * xAxis[0] - zAxis[0] * xAxis[2],
    zAxis[0] * xAxis[1] - zAxis[1] * xAxis[0],
  ];
  const R = [0, 1, 2].map((r) => [xAxis[r], yAxis[r], zAxis[r]]);
  return { fx: f, fy: f, cx: w / 2, cy: h / 2, w, h, R, t: [sgn * (rover.length / 2), 0, 0.5], est: true };
}

export class CameraDeck {
  constructor(S) {
    this.S = S;
    this.main = prefs.get("mainCam", "front");
    this.auto = prefs.get("autoRev", false);
    this.guides = prefs.get("guides", true);
    this.manualMain = this.main;
    this.revSince = null;
    this.fwdSince = null;

    this.stage = { main: document.getElementById("main-stage"), corner: document.getElementById("corner-stage") };
    this.imgs = {};
    this.lastLoad = {};
    for (const k of KEYS) {
      const img = new Image();
      img.alt = `${k} camera`;
      img.decoding = "async";
      img.addEventListener("error", () => this.scheduleReload(k));
      this.imgs[k] = img;
      this.load(k);
    }
    // The placeholder <img> tags in the HTML are replaced by the live ones.
    document.getElementById("main-img").remove();
    document.getElementById("corner-img").remove();

    this.overlay = new Surface(document.getElementById("main-overlay"), (ctx, w, h) => this.drawMain(ctx, w, h));
    this.cornerOverlay = new Surface(document.getElementById("corner-overlay"), (ctx, w, h) => this.drawCorner(ctx, w, h));

    document.getElementById("corner-view").addEventListener("click", () => this.swap());
    document.getElementById("mode-guides").addEventListener("click", () => this.toggleGuides());
    document.getElementById("mode-auto").addEventListener("click", () => this.toggleAuto());
    this.place();
  }

  load(k) {
    this.lastLoad[k] = performance.now();
    this.imgs[k].src = `/cam/${k}.mjpg?t=${Date.now()}`;
  }

  scheduleReload(k) {
    clearTimeout(this["reload_" + k]);
    this["reload_" + k] = setTimeout(() => this.load(k), 1500);
  }

  reconnectAll() {
    for (const k of KEYS) this.load(k);
  }

  get corner() {
    return this.main === "front" ? "rear" : "front";
  }

  /** Move the live <img> elements into their panes. Re-parenting keeps the
   * MJPEG connection open, so a swap is instant. */
  place() {
    this.stage.main.prepend(this.imgs[this.main]);
    this.stage.corner.prepend(this.imgs[this.corner]);
    document.getElementById("main-label").textContent = `CAM-${NUM[this.main]} ${this.main.toUpperCase()}${this.auto && this.main !== this.manualMain ? " · AUTO" : ""}`;
    document.getElementById("corner-label").textContent = `CAM-${NUM[this.corner]} ${this.corner.toUpperCase()}`;
    document.getElementById("mode-guides").classList.toggle("on", this.guides);
    document.getElementById("mode-auto").classList.toggle("on", this.auto);
  }

  swap() {
    this.main = this.corner;
    this.manualMain = this.main;
    prefs.set("mainCam", this.main);
    this.place();
  }

  toggleAuto() {
    this.auto = !this.auto;
    prefs.set("autoRev", this.auto);
    if (!this.auto) this.main = this.manualMain;
    this.place();
  }

  toggleGuides() {
    this.guides = !this.guides;
    prefs.set("guides", this.guides);
    this.place();
  }

  onTele() {
    const S = this.S;
    const t = S.tele;
    // Restart a feed that the server says is live but the <img> stopped
    // showing (e.g. the stream connection died during a Wi-Fi drop).
    for (const k of KEYS) {
      const cam = t.cams?.[k];
      const fresh = cam && isNum(cam.age) && cam.age < 1;
      const img = this.imgs[k];
      if (fresh && !img.naturalWidth && performance.now() - this.lastLoad[k] > 4000) this.load(k);
    }

    if (!this.auto) return;
    const now = performance.now() / 1000;
    const vx = S.derived.cmd ? S.derived.cmdVx : 0;
    if (vx < -0.03) {
      this.revSince ??= now;
      this.fwdSince = null;
    } else if (vx > 0.03) {
      this.fwdSince ??= now;
      this.revSince = null;
    }
    let want = this.main;
    if (this.revSince !== null && now - this.revSince > AUTO_HOLD_S) want = "rear";
    if (this.fwdSince !== null && now - this.fwdSince > AUTO_HOLD_S) want = "front";
    if (want !== this.main) {
      this.main = want;
      this.place();
    }
  }

  camState(k) {
    const cam = this.S.tele?.cams?.[k];
    if (!this.S.connected) return { live: false, why: "HUD server offline" };
    if (!cam || cam.age === null) return { live: false, why: `nothing received on ${this.S.config?.cams?.[k]?.topic ?? k}` };
    if (cam.age > 1.5) return { live: false, why: `last frame ${cam.age.toFixed(1)} s ago` };
    return { live: true, cam };
  }

  frame() {
    for (const [pane, k] of [["main", this.main], ["corner", this.corner]]) {
      const st = this.camState(k);
      const ns = document.getElementById(`${pane}-nosig`);
      ns.classList.toggle("show", !st.live);
      document.getElementById(`${pane}-nosig-detail`).textContent = st.why || "";
      const meta = document.getElementById(`${pane}-meta`);
      if (st.live) {
        const c = st.cam;
        const size = c.size ? `${c.size[0]}×${c.size[1]} · ` : "";
        meta.textContent = pane === "main"
          ? `${size}${c.hz.toFixed(0)} FPS · ${(c.bps / 1024).toFixed(0)} KB/S`
          : `${c.hz.toFixed(0)} FPS`;
      } else meta.textContent = "— FPS";
    }
    this.overlay.frame();
    this.cornerOverlay.frame();
  }

  /** Displayed image rectangle inside the pane (object-fit: contain). */
  imageRect(img, w, h, model) {
    const iw = img.naturalWidth || model?.w || 16;
    const ih = img.naturalHeight || model?.h || 10;
    const s = Math.min(w / iw, h / ih);
    return { x: (w - iw * s) / 2, y: (h - ih * s) / 2, w: iw * s, h: ih * s, iw, ih };
  }

  drawCorner(ctx, w, h) {
    ctx.strokeStyle = C.orangeDim;
    ctx.lineWidth = 1;
    const s = Math.min(w, h) * 0.04;
    ctx.beginPath();
    ctx.moveTo(w / 2 - s, h / 2);
    ctx.lineTo(w / 2 + s, h / 2);
    ctx.moveTo(w / 2, h / 2 - s);
    ctx.lineTo(w / 2, h / 2 + s);
    ctx.stroke();
  }

  drawMain(ctx, w, h) {
    const S = this.S;
    const k = this.main;
    const live = this.camState(k).live;
    const rover = S.config?.rover;
    const model = S.config?.cams?.[k]?.model || (rover ? fallbackModel(k, rover) : null);
    const r = this.imageRect(this.imgs[k], w, h, model);

    // Corner brackets around the actual image area.
    ctx.strokeStyle = C.orange;
    ctx.lineWidth = 2;
    glow(ctx, C.orange, 6);
    const b = Math.min(r.w, r.h) * 0.06;
    const ins = 10;
    ctx.beginPath();
    for (const [x, y, dx, dy] of [
      [r.x + ins, r.y + ins, 1, 1], [r.x + r.w - ins, r.y + ins, -1, 1],
      [r.x + ins, r.y + r.h - ins, 1, -1], [r.x + r.w - ins, r.y + r.h - ins, -1, -1],
    ]) {
      ctx.moveTo(x, y + dy * b);
      ctx.lineTo(x, y);
      ctx.lineTo(x + dx * b, y);
    }
    ctx.stroke();
    noGlow(ctx);

    if (live && this.guides && model && rover) this.drawGuides(ctx, r, model, rover, k);
    this.drawReticle(ctx, r);
    this.drawHeadingTape(ctx, r, k);
    this.drawSpeed(ctx, r, k);
    if (k === "rear") {
      label(ctx, "◀ REAR VIEW · IMAGE LEFT = ROVER RIGHT ▶", r.x + r.w / 2, r.y + r.h - 16, { size: 12, color: C.amber, align: "center" });
    }
  }

  drawReticle(ctx, r) {
    const cx = r.x + r.w / 2, cy = r.y + r.h / 2;
    const s = Math.min(r.w, r.h) * 0.035;
    ctx.strokeStyle = "rgba(255,138,30,0.75)";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(cx - s * 2, cy); ctx.lineTo(cx - s * 0.6, cy);
    ctx.moveTo(cx + s * 0.6, cy); ctx.lineTo(cx + s * 2, cy);
    ctx.moveTo(cx, cy - s * 0.6); ctx.lineTo(cx, cy - s * 1.4);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(cx, cy, 2, 0, Math.PI * 2);
    ctx.fillStyle = C.orange;
    ctx.fill();
  }

  /** Heading tape along the top edge; map-frame yaw, CCW degrees like ROS. */
  drawHeadingTape(ctx, r, k) {
    const pose = this.S.derived.pose;
    if (!pose || this.S.derived.poseStale || !isNum(pose.yaw)) return;
    let hdg = deg(pose.yaw) + (k === "rear" ? 180 : 0);
    hdg = ((hdg % 360) + 360) % 360;
    const cx = r.x + r.w / 2;
    const y = r.y + 46;
    const span = Math.min(560, r.w * 0.5);
    const pxPerDeg = span / 90; // 90° visible
    ctx.save();
    ctx.beginPath();
    ctx.rect(cx - span / 2, y - 26, span, 40);
    ctx.clip();
    ctx.fillStyle = "rgba(0,0,0,0.45)";
    ctx.fillRect(cx - span / 2, y - 26, span, 40);
    ctx.strokeStyle = C.orange;
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    // Positive yaw is CCW (left), so larger headings sit to the left.
    for (let d = Math.floor(hdg - 50); d <= hdg + 50; d++) {
      if (d % 5) continue;
      const x = cx - (d - hdg) * pxPerDeg;
      const major = d % 30 === 0;
      ctx.moveTo(x, y + 8);
      ctx.lineTo(x, y + 8 - (major ? 12 : d % 10 === 0 ? 7 : 4));
      if (major && Math.abs(x - cx) < span / 2 - 16) {
        const v = ((d % 360) + 360) % 360;
        label(ctx, String(v).padStart(3, "0"), x, y - 9, { size: 11, color: C.text, align: "center", font: FONT_MONO, weight: 400 });
      }
    }
    ctx.stroke();
    ctx.restore();
    // Index box.
    ctx.fillStyle = "#000";
    ctx.strokeStyle = C.orange;
    ctx.lineWidth = 1.5;
    glow(ctx, C.orange, 6);
    ctx.beginPath();
    ctx.rect(cx - 30, y - 30, 60, 20);
    ctx.fill();
    ctx.stroke();
    noGlow(ctx);
    label(ctx, hdg.toFixed(0).padStart(3, "0") + "°", cx, y - 15, { size: 15, color: "#fff3e0", align: "center", font: FONT_MONO });
    ctx.beginPath();
    ctx.moveTo(cx, y + 10);
    ctx.lineTo(cx - 5, y + 16);
    ctx.lineTo(cx + 5, y + 16);
    ctx.closePath();
    ctx.fillStyle = C.orange;
    ctx.fill();
    label(ctx, "HDG", cx - span / 2 - 6, y + 6, { size: 10, color: C.textDim, align: "right" });
  }

  drawSpeed(ctx, r, k) {
    const d = this.S.derived;
    const x = r.x + 26;
    const y = r.y + r.h - 96;
    if (r.w < 500) return;
    label(ctx, "GROUND SPEED", x, y - 30, { size: 10, color: C.textDim });
    glow(ctx, C.orange, 8);
    label(ctx, isNum(d.vx) ? fmtSigned(d.vx, 2) : "—", x, y, { size: 30, color: "#fff3e0", font: FONT_MONO });
    noGlow(ctx);
    label(ctx, "M/S", x + 112, y, { size: 11, color: C.textDim });
    // Direction chevron: which way the rover is going relative to this camera.
    const moving = isNum(d.vx) && Math.abs(d.vx) > 0.03;
    if (moving) {
      const towards = (d.vx > 0) === (k === "front");
      label(ctx, towards ? "▲ TOWARDS VIEW" : "▼ AWAY FROM VIEW", x, y + 20, { size: 11, color: towards ? C.green : C.amber });
    }
  }

  /** Predicted footprint sweep on the ground, projected into the image. */
  drawGuides(ctx, r, m, rover, k) {
    const d = this.S.derived;
    const sgn = k === "front" ? 1 : -1;
    // Curvature from what the rover is being told to do; measured motion as a
    // fallback so autonomy turns also bend the guides.
    let v = d.cmd ? d.cmdVx : d.vx ?? 0;
    let wz = d.cmd ? d.cmdWz : d.wz ?? 0;
    let kappa = Math.abs(v) > 0.05 ? wz / v : 0;
    // Tighter than about the rover's own size, or turning on the spot, the
    // swept arc says nothing useful from a forward camera: show straight
    // distance guides, dimmed, and a PIVOT cue instead.
    const pivot = Math.abs(kappa) > PIVOT_KAPPA || (Math.abs(v) <= 0.05 && Math.abs(wz) > 0.1);
    if (pivot) kappa = 0;
    const half = rover.width / 2;
    const nose = rover.length / 2;
    // Display px per camera_info px (the stream may be scaled vs. the info).
    const scale = r.w / m.w;

    const project = (bx, by) => {
      const px = bx - m.t[0], py = by - m.t[1], pz = 0 - m.t[2];
      const R = m.R; // optical->base rows; use transpose for base->optical
      const X = R[0][0] * px + R[1][0] * py + R[2][0] * pz;
      const Y = R[0][1] * px + R[1][1] * py + R[2][1] * pz;
      const Z = R[0][2] * px + R[1][2] * py + R[2][2] * pz;
      if (Z < 0.15) return null;
      const u = (m.fx * X) / Z + m.cx;
      const vv = (m.fy * Y) / Z + m.cy;
      return [r.x + u * scale, r.y + vv * scale];
    };
    // Pose after travelling arc length s (signed) along curvature kappa,
    // applied to a body-frame point (px, py).
    const along = (s, px, py) => {
      const th = kappa * s;
      let x, y;
      if (Math.abs(kappa) < 1e-4) { x = s; y = 0; } else { x = Math.sin(th) / kappa; y = (1 - Math.cos(th)) / kappa; }
      const c = Math.cos(th), sn = Math.sin(th);
      return [x + c * px - sn * py, y + sn * px + c * py];
    };

    // Cap the drawn sweep at a quarter turn: when nearly spinning on the spot
    // a full-length arc would curl out of the image and the guides vanish.
    const len = Math.abs(kappa) > 1e-3 ? Math.min(GUIDE_LEN, Math.PI / 2 / Math.abs(kappa)) : GUIDE_LEN;
    const steps = 28;
    const edge = (side) => {
      const pts = [];
      for (let i = 0; i <= steps; i++) {
        const s = sgn * (len * i) / steps;
        const [bx, by] = along(s, sgn * nose, side * half);
        const p = project(bx, by);
        if (p) pts.push(p);
      }
      return pts;
    };

    ctx.save();
    ctx.beginPath();
    ctx.rect(r.x, r.y, r.w, r.h);
    ctx.clip();
    if (pivot) ctx.globalAlpha = 0.45;
    const L = edge(1), Rr = edge(-1);
    // Swept corridor fill.
    if (L.length > 1 && Rr.length > 1) {
      ctx.beginPath();
      L.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
      [...Rr].reverse().forEach(([x, y]) => ctx.lineTo(x, y));
      ctx.closePath();
      ctx.fillStyle = "rgba(61,255,122,0.07)";
      ctx.fill();
    }
    ctx.lineWidth = 3;
    glow(ctx, C.green, 6);
    for (const pts of [L, Rr]) {
      if (pts.length < 2) continue;
      // Colour the rails by distance band.
      for (let i = 1; i < pts.length; i++) {
        const dist = (len * i) / steps;
        ctx.strokeStyle = dist <= 0.5 ? C.red : dist <= 1.0 ? C.amber : C.green;
        ctx.beginPath();
        ctx.moveTo(...pts[i - 1]);
        ctx.lineTo(...pts[i]);
        ctx.stroke();
      }
    }
    // Distance rungs.
    ctx.lineWidth = 2;
    for (const [dist, col] of GUIDE_TICKS) {
      if (dist > len + 1e-6) continue;
      const a = project(...along(sgn * dist, sgn * nose, half));
      const b = project(...along(sgn * dist, sgn * nose, -half));
      if (!a || !b) continue;
      ctx.strokeStyle = col;
      glow(ctx, col, 6);
      ctx.beginPath();
      // Short rungs from each rail, like a parking guide, with a gap in the middle.
      const f = 0.22;
      ctx.moveTo(...a);
      ctx.lineTo(a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f);
      ctx.moveTo(...b);
      ctx.lineTo(b[0] + (a[0] - b[0]) * f, b[1] + (a[1] - b[1]) * f);
      ctx.stroke();
      noGlow(ctx);
      label(ctx, `${dist.toFixed(1)}m`, b[0] + 6 * Math.sign(b[0] - a[0] || 1), b[1] + 4, { size: 11, color: col, font: FONT_MONO, align: b[0] > a[0] ? "left" : "right" });
    }
    noGlow(ctx);
    ctx.restore();
    if (pivot) {
      // +wz is CCW = rover turning left, whichever camera is in front.
      const left = wz > 0;
      const cx = r.x + r.w / 2, cy = r.y + r.h * 0.72;
      ctx.strokeStyle = C.amber;
      ctx.lineWidth = 3;
      glow(ctx, C.amber, 8);
      ctx.beginPath();
      const a0 = left ? -0.25 * Math.PI : -0.75 * Math.PI, a1 = left ? -0.75 * Math.PI : -0.25 * Math.PI;
      ctx.arc(cx, cy + 40, 70, a0, a1, left);
      ctx.stroke();
      const ex = cx + Math.cos(a1) * 70, ey = cy + 40 + Math.sin(a1) * 70;
      ctx.fillStyle = C.amber;
      ctx.beginPath();
      ctx.moveTo(ex + (left ? -10 : 10), ey + 2);
      ctx.lineTo(ex + (left ? 6 : -6), ey - 10);
      ctx.lineTo(ex + (left ? 6 : -6), ey + 12);
      ctx.closePath();
      ctx.fill();
      noGlow(ctx);
      label(ctx, `PIVOT ${left ? "LEFT" : "RIGHT"}`, cx, cy + 4, { size: 15, color: C.amber, align: "center" });
    }
    if (m.est) label(ctx, "GUIDES ESTIMATED · NO CAMERA_INFO/TF", r.x + r.w - 16, r.y + r.h - 36, { size: 10, color: C.textDim, align: "right" });
  }
}
