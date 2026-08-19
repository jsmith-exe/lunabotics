#!/usr/bin/env python3
"""Aggregate dead-reckoning runs into a VO-on / VO-off comparison."""
import glob, json, os, sys, statistics as st

outdir = sys.argv[1]
groups = {}
for f in sorted(glob.glob(os.path.join(outdir, "*.json"))):
    d = json.load(open(f))
    label = d["label"]
    cond = label.rsplit("_", 1)[0]
    groups.setdefault(cond, []).append(d)

def col(runs, src, key):
    vals = []
    for r in runs:
        s = r["sources"].get(src)
        if s and s.get(key) is not None:
            vals.append(s[key])
    return vals

def fmt(vals, prec=3):
    if not vals:
        return "     -    "
    if len(vals) == 1:
        return f"{vals[0]:.{prec}f}"
    return f"{st.mean(vals):.{prec}f}±{st.stdev(vals):.{prec}f}"

print(f"\n{'cond':<10} {'n':<3} {'path_m':<14} {'src':<7} "
      f"{'final_err_m':<16} {'%path':<14} {'max_m':<14} {'final_yaw_deg':<14}")
print("-" * 100)
for cond, runs in sorted(groups.items()):
    paths = [r["truth_path_len_m"] for r in runs]
    for src in ("wheel", "vo", "fused"):
        if not col(runs, src, "final_pos_err_m"):
            continue
        print(f"{cond:<10} {len(runs):<3} {fmt(paths):<14} {src:<7} "
              f"{fmt(col(runs,src,'final_pos_err_m')):<16} "
              f"{fmt(col(runs,src,'final_pos_err_pct_of_path'),2):<14} "
              f"{fmt(col(runs,src,'max_pos_err_m')):<14} "
              f"{fmt(col(runs,src,'final_yaw_err_deg'),2):<14}")
    print()

# The headline number: fused estimate, VO off vs VO on.
for a, b in (("novo", "vo"), ("novo_slip", "vo_slip")):
    if a in groups and b in groups:
        fa = col(groups[a], "fused", "final_pos_err_m")
        fb = col(groups[b], "fused", "final_pos_err_m")
        if fa and fb:
            ma, mb = st.mean(fa), st.mean(fb)
            print(f"{a} -> {b}: fused final drift {ma:.3f} m -> {mb:.3f} m "
                  f"({100*(ma-mb)/ma:+.1f}% change, {ma/mb:.2f}x better)"
                  if mb > 0 else "")
