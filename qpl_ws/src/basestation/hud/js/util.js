// Shared drawing helpers and palette for the HUD canvases.

export const C = {
  orange: "#ff8a1e",
  orangeDim: "rgba(255,138,30,0.35)",
  orangeFaint: "rgba(255,138,30,0.12)",
  amber: "#ffc23a",
  red: "#ff2d1f",
  green: "#3dff7a",
  greenDim: "rgba(61,255,122,0.30)",
  greenFaint: "rgba(61,255,122,0.12)",
  cyan: "#48f0d8",
  purple: "#9b6bff",
  text: "#ffd9a8",
  textDim: "#b0702c",
  grey: "#6d5a48",
};

export const FONT_LABEL = '"Nimbus Sans Narrow","Liberation Sans Narrow","Ubuntu Condensed","Arial Narrow",sans-serif';
export const FONT_MONO = '"DejaVu Sans Mono","Ubuntu Mono",monospace';
export const FONT_JP = '"Noto Serif CJK JP","Noto Sans CJK JP",serif';

// Ages (s) at which a stream counts as late / gone.
export const STALE_S = 0.6;
export const DEAD_S = 2.0;
// Tilt thresholds (deg). Regolith slopes in the pit are the realistic risk.
export const TILT_CAUTION = 12;
export const TILT_CRIT = 20;

export const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
export const deg = (r) => (r * 180) / Math.PI;
export const isNum = (v) => typeof v === "number" && Number.isFinite(v);

/** Format a signed number with fixed decimals and an explicit + sign. */
export function fmtSigned(v, d = 2) {
  if (!isNum(v)) return "—";
  const s = Math.abs(v).toFixed(d);
  return (v < 0 && Number(s) !== 0 ? "−" : "+") + s;
}

/**
 * Canvas that tracks its CSS box and the device pixel ratio. draw(ctx, w, h)
 * gets CSS-pixel units. Call .frame() from the render loop.
 */
export class Surface {
  constructor(canvas, draw) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.draw = draw;
    this.w = 0;
    this.h = 0;
    const ro = new ResizeObserver(() => this.resize());
    ro.observe(canvas);
    this.resize();
  }
  resize() {
    const r = this.canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.w = Math.max(1, r.width);
    this.h = Math.max(1, r.height);
    this.canvas.width = Math.round(this.w * dpr);
    this.canvas.height = Math.round(this.h * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  frame() {
    const { ctx, w, h } = this;
    ctx.clearRect(0, 0, w, h);
    ctx.save();
    this.draw(ctx, w, h);
    ctx.restore();
  }
}

export function glow(ctx, color, blur = 6) {
  ctx.shadowColor = color;
  ctx.shadowBlur = blur;
}
export function noGlow(ctx) {
  ctx.shadowBlur = 0;
  ctx.shadowColor = "transparent";
}

export function label(ctx, text, x, y, { size = 11, color = C.textDim, align = "left", base = "alphabetic", font = FONT_LABEL, weight = 700 } = {}) {
  ctx.font = `${weight} ${size}px ${font}`;
  ctx.fillStyle = color;
  ctx.textAlign = align;
  ctx.textBaseline = base;
  ctx.fillText(text, x, y);
}

/** Small "+" graticule mark, the signature of the NERV plot screens. */
export function cross(ctx, x, y, s = 4) {
  ctx.moveTo(x - s, y);
  ctx.lineTo(x + s, y);
  ctx.moveTo(x, y - s);
  ctx.lineTo(x, y + s);
}

/** Rounded rectangle boxed label (canvas version of the .tag callouts). */
export function boxLabel(ctx, text, x, y, { size = 11, color = C.orange, bg = "rgba(0,0,0,0.78)", align = "center", pad = 4 } = {}) {
  ctx.font = `700 ${size}px ${FONT_LABEL}`;
  const w = ctx.measureText(text).width + pad * 2;
  const h = size + pad * 1.6;
  let x0 = x;
  if (align === "center") x0 = x - w / 2;
  else if (align === "right") x0 = x - w;
  const y0 = y - h / 2;
  ctx.fillStyle = bg;
  roundRect(ctx, x0, y0, w, h, 3);
  ctx.fill();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.2;
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  ctx.fillText(text, x0 + pad, y + 0.5);
  return w;
}

export function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

/** Persisted per-browser UI preference; never required for correctness. */
export const prefs = {
  get(key, fallback) {
    try {
      const v = localStorage.getItem("qplhud." + key);
      return v === null ? fallback : JSON.parse(v);
    } catch {
      return fallback;
    }
  },
  set(key, value) {
    try {
      localStorage.setItem("qplhud." + key, JSON.stringify(value));
    } catch {
      /* private mode etc. */
    }
  },
};
