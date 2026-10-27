#!/usr/bin/env python3
"""Fit GaggiMate's pump model (pumpModelCoeffs / pumpSlipCoeffs) from flow-meter shot logs (fork).

Input: the meter board's shot CSVs (meter v1.7.3+ with a Pro on meterlink.4+, so every row carries the Pro's
pump power and pressure next to the metered pulses). Typically the Cal2 runs: [Cal2 0] with the needle valve
open, then [Cal2 1] + [Cal2 2] at several valve openings.

The firmware's model (lib/NayrodPID/src/PressureController/PressureController.cpp):

    S(P) = max(0, slip polynomial)            pumpSlipCoeffs  a,b,c,d  ->  a*P^3 + b*P^2 + c*P + d
    A(P) = full-drive polynomial              pumpModelCoeffs a,b,c,d  ->  same form
    Q    = max(0, d * (A(P) + S(P)) - S(P))   d = pump power / 100, P = pressure the Pro reads, Q in ml/s

so at each pressure Q is a straight line in pump power: slope G(P) = A(P) + S(P), zero at d = S/G.
The stock pressure loop only uses G(P) (it scales the convergence term); a pressure phase with a flow
limit uses the whole model.

What the script does:
  1. Cuts each log into steps (power phases: constant pump power; flow phases: constant targets).
  2. Takes the last --window seconds of each step and keeps it only if it has settled (pressure slope,
     flow first half vs second half, pump power spread). Flow is the pulse count over the window.
  3. Fits Q = d*G(P) - S(P) by least squares, for G and S of degree 0..3, and for G held constant
     (the structure of the Oct 20 tune, which keeps the pressure loop's gain the same at every pressure).
  4. Scores every variant by cross-validation (leave one log out), so the pick is the one that predicts
     logs it has not seen, not the one that bends best through the points it was fitted on.
  5. Prints the coefficients to paste, the error per point with the coefficients as printed (rounded,
     evaluated exactly like the firmware), the same for the current coefficients, the meter vs BOOKOO
     check per point and the convergence gain that keeps today's pressure-loop gain.

Run: python3 fit_pump_model.py flowmeter_all.csv   (or GM_*.csv) [--current-flow a,b,c,d --current-slip a,b,c,d]
"""
import argparse
import csv
import itertools
import math
import os
import sys

import numpy as np

CUR_FLOW = "-0.00220,0.03818,-1.3725,23.655"  # Oct 20 2026 tune
CUR_SLIP = "0.00220,-0.03818,1.3725,6.325"
P_EVAL_MAX = 11.0


# ----------------------------------------------------------------------------------------------- reading
def read_logs(path):
    """One shot log (GM_....csv), or the meter page's "download all" CSV (flowmeter_all.csv, one shot per
    value of its first column). Returns [(name, pulses_per_litre, columns)]."""
    meta, lines = {}, []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("#"):
                k, _, v = line[1:].strip().partition("=")
                meta[k.strip()] = v.strip()
            elif line.strip():
                lines.append(line)
    rows = list(csv.DictReader(lines))
    if not rows:
        raise ValueError("empty")
    if "shot" in rows[0] and "pulses_per_litre" in rows[0]:  # all logs in one file
        shots = {}
        for r in rows:
            shots.setdefault(r["shot"], []).append(r)
        out = []
        for name, rs in shots.items():
            if not any(r.get("pump_pct") not in ("", None) for r in rs):
                continue  # logged before meter v1.7.3 / Pro meterlink.4: no pump columns
            out.append((name, float(rs[0]["pulses_per_litre"]), columns(rs)))
        if not out:
            raise ValueError("no shot with pump_pct values (needs meter v1.7.3+ and Pro meterlink.4+)")
        return out
    if "pump_pct" not in rows[0]:
        raise ValueError("no pump_pct column (needs meter v1.7.3+ and Pro meterlink.4+)")
    ppl = float(meta.get("pulses_per_litre", "nan"))
    if not ppl > 0:
        raise ValueError("no pulses_per_litre in the header")
    return [(os.path.basename(path), ppl, columns(rows))]


def columns(rows):
    def num(r, k):
        v = r.get(k, "")
        try:
            return float(v) if v not in ("", None) else math.nan
        except ValueError:
            return math.nan

    cols = ["t_ms", "pulses", "cup_g", "pump_pct", "pressure_bar", "pro_mode", "target_bar", "target_ml_s", "pro_flags"]
    data = {k: np.array([num(r, k) for r in rows]) for k in cols}
    data["t"] = data["t_ms"] / 1000.0
    return data


# ------------------------------------------------------------------------------------------ segmentation
def segments(data):
    """Runs of rows with the same step key: (mode, targets) and, in power mode, the pump power."""
    n = len(data["t"])
    keys = []
    for i in range(n):
        pump = data["pump_pct"][i]
        flags = data["pro_flags"][i]
        if math.isnan(pump) or math.isnan(flags) or not (int(flags) & 1):
            keys.append(None)  # no $D, or brew valve closed
            continue
        mode = int(data["pro_mode"][i])
        key = (mode, round(data["target_bar"][i], 1), round(data["target_ml_s"][i], 2))
        if mode == 0:
            key += (round(pump, 1),)
        keys.append(key)
    segs, start = [], 0
    for i in range(1, n + 1):
        if i == n or keys[i] != keys[start]:
            if keys[start] is not None:
                segs.append((start, i - 1, keys[start]))
            start = i
    return segs


def step_point(data, ppl, i0, i1, a):
    """Steady-state point from the last `window` seconds of rows i0..i1, or None + reason."""
    t = data["t"]
    t_end = t[i1] - a.end_margin
    t_beg = t_end - a.window
    if t_beg < t[i0] + a.min_settle:
        return None, "step too short"
    idx = np.where((t >= t_beg) & (t <= t_end))[0]
    if len(idx) < 0.6 * a.window * 10:
        return None, "gap in the log"
    tw, nw = t[idx], data["pulses"][idx]
    pw, prw = data["pump_pct"][idx], data["pressure_bar"][idx]
    dt = tw[-1] - tw[0]
    q = (nw[-1] - nw[0]) / ppl * 1000.0 / dt
    mid = idx[len(idx) // 2]
    q1 = (data["pulses"][mid] - nw[0]) / ppl * 1000.0 / (t[mid] - tw[0])
    q2 = (nw[-1] - data["pulses"][mid]) / ppl * 1000.0 / (tw[-1] - t[mid])
    p_slope = np.polyfit(tw - tw[0], prw, 1)[0]
    pt = dict(d=float(np.mean(pw)) / 100.0, P=float(np.mean(prw)), Q=float(q), pulses=int(nw[-1] - nw[0]),
              p_slope=float(p_slope), dq=float(q2 - q1), pump_sd=float(np.std(pw)), t0=float(tw[0]), t1=float(tw[-1]))
    cup = data["cup_g"][idx]
    ok = ~np.isnan(cup)
    pt["cup"] = float(np.polyfit(tw[ok] - tw[0], cup[ok], 1)[0]) if ok.sum() >= 0.8 * len(idx) else math.nan
    why = []
    if abs(p_slope) > a.max_p_slope:
        why.append(f"pressure still moving {p_slope:+.3f} bar/s")
    if abs(q2 - q1) > max(a.max_dq_rel * q, a.max_dq_abs):
        why.append(f"flow still moving {q1:.2f}->{q2:.2f}")
    if pt["pump_sd"] > a.max_pump_sd:
        why.append(f"pump power spread {pt['pump_sd']:.1f}")
    if pt["P"] > a.p_max:
        why.append(f"pressure {pt['P']:.2f} > {a.p_max}")
    return pt, "; ".join(why)


# ----------------------------------------------------------------------------------------------- model
def poly(c, P):  # firmware order a,b,c,d -> a P^3 + b P^2 + c P + d
    return ((c[0] * P + c[1]) * P + c[2]) * P + c[3]


def firmware_q(flow_c, slip_c, d, P):
    s = np.maximum(0.0, poly(slip_c, P))
    return np.maximum(0.0, d * (poly(flow_c, P) + s) - s)


def design(d, P, deg_g, deg_s, const_g):
    cols = [d] if const_g else [d * P ** k for k in range(deg_g + 1)]
    cols += [-(P ** k) for k in range(deg_s + 1)]
    return np.column_stack(cols)


def to_coeffs(theta, deg_g, deg_s, const_g):
    g = [theta[0]] if const_g else list(theta[: deg_g + 1])
    s = list(theta[len(g):])
    g += [0.0] * (4 - len(g))
    s += [0.0] * (4 - len(s))
    G = [g[3], g[2], g[1], g[0]]  # firmware order
    S = [s[3], s[2], s[1], s[0]]
    A = [G[i] - S[i] for i in range(4)]
    return A, S


def fit(pts, deg_g, deg_s, const_g):
    d = np.array([p["d"] for p in pts])
    P = np.array([p["P"] for p in pts])
    Q = np.array([p["Q"] for p in pts])
    X = design(d, P, deg_g, deg_s, const_g)
    theta, *_ = np.linalg.lstsq(X, Q, rcond=None)
    return to_coeffs(theta, deg_g, deg_s, const_g)


def rounded(c):
    return [float(f"{x:.6g}") for x in c]


def fmt(c):
    return ",".join(f"{x:.6g}" for x in c)


def errors(A, S, pts):
    d = np.array([p["d"] for p in pts])
    P = np.array([p["P"] for p in pts])
    Q = np.array([p["Q"] for p in pts])
    return firmware_q(A, S, d, P) - Q


def variants():
    for deg_s in range(4):
        yield ("G const", 0, deg_s, True)
        for deg_g in range(1, 4):
            yield (f"G deg {deg_g}", deg_g, deg_s, False)


def n_params(deg_g, deg_s, const_g):
    return (1 if const_g else deg_g + 1) + deg_s + 1


def cross_val(pts, groups, deg_g, deg_s, const_g):
    """Leave one log out (or one point out with fewer than 3 logs). Returns RMS and max error, ml/s."""
    errs = []
    keys = sorted(set(groups))
    folds = [[i for i, g in enumerate(groups) if g == k] for k in keys] if len(keys) >= 3 else [[i] for i in range(len(pts))]
    for test in folds:
        train = [p for i, p in enumerate(pts) if i not in test]
        if len(train) < n_params(deg_g, deg_s, const_g) + 2:
            return math.inf, math.inf
        A, S = fit(train, deg_g, deg_s, const_g)
        errs += list(errors(A, S, [pts[i] for i in test]))
    e = np.array(errs)
    return float(np.sqrt(np.mean(e ** 2))), float(np.max(np.abs(e)))


def sane(A, S):
    P = np.linspace(0.0, P_EVAL_MAX, 111)
    s = poly(S, P)
    g = poly(A, P) + np.maximum(0.0, s)
    notes = []
    if np.min(s) < -0.05:
        notes.append(f"slip < 0 at {P[np.argmin(s)]:.1f} bar (firmware clamps it to 0)")
    if np.min(g) <= 0.5:
        notes.append(f"G(P) <= 0.5 ml/s at {P[np.argmin(g)]:.1f} bar")
    return notes


# ------------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--window", type=float, default=4.0, help="seconds at the end of each step to average (4)")
    ap.add_argument("--end-margin", type=float, default=0.3, help="seconds left out before a step ends (0.3)")
    ap.add_argument("--min-settle", type=float, default=3.0, help="seconds after a step starts before the window (3)")
    ap.add_argument("--max-p-slope", type=float, default=0.05, help="bar/s allowed in the window (0.05)")
    ap.add_argument("--max-dq-rel", type=float, default=0.05, help="flow change first->second half, share (0.05)")
    ap.add_argument("--max-dq-abs", type=float, default=0.08, help="... or ml/s, whichever is larger (0.08)")
    ap.add_argument("--max-pump-sd", type=float, default=1.5, help="pump power spread in the window, %% (1.5)")
    ap.add_argument("--p-max", type=float, default=10.4, help="leave out points above this pressure (10.4)")
    ap.add_argument("--q-min", type=float, default=0.5, help="leave out points below this flow from the fit (0.5)")
    ap.add_argument("--current-flow", default=CUR_FLOW)
    ap.add_argument("--current-slip", default=CUR_SLIP)
    ap.add_argument("--convergence", type=float, default=3.5, help="convergence gain in use (3.5)")
    ap.add_argument("--csv-out", help="write the points with model errors to this CSV")
    a = ap.parse_args()

    pts, groups, rejected = [], [], []
    logs = []
    for path in a.logs:
        try:
            logs += read_logs(path)
        except ValueError as e:
            print(f"skip {os.path.basename(path)}: {e}")
    for name, ppl, data in logs:
        for i0, i1, key in segments(data):
            pt, why = step_point(data, ppl, i0, i1, a)
            label = f"{name[:22]} mode {key[0]} " + (f"{key[3]:.0f}%" if key[0] == 0 else f"{key[1]:.1f} bar/{key[2]:.1f} ml/s")
            if pt is None:
                continue
            pt["label"] = label
            pt["file"] = name
            if why:
                rejected.append((label, pt, why))
            elif pt["Q"] < a.q_min:
                rejected.append((label, pt, f"flow {pt['Q']:.2f} < {a.q_min} (kept for the stall check)"))
            else:
                pts.append(pt)
                groups.append(name)

    print(f"\n{len(pts)} settled points from {len(set(groups))} logs, {len(rejected)} left out\n")
    for label, pt, why in rejected:
        print(f"  left out  {label:<48} d {pt['d']*100:5.1f}%  P {pt['P']:5.2f}  Q {pt['Q']:5.2f}   {why}")
    if len(pts) < 6:
        sys.exit("\nnot enough settled points to fit")

    # BOOKOO vs meter (cup g/s over the same window; 1 ml of tank water ~ 0.998 g)
    ratios = [p["cup"] / (0.998 * p["Q"]) for p in pts if not math.isnan(p["cup"]) and p["Q"] > 0.3]
    if ratios:
        r = np.array(ratios)
        print(f"\nBOOKOO / meter over the same windows: mean {r.mean():.3f}, spread {r.std():.3f}, {len(r)} points "
              f"(1.000 = the meter's pulses per litre matches the scale)")

    # model choice by cross-validation
    print("\nvariant           params  fit RMS   cross-val RMS / max   (ml/s)")
    results = []
    for name, dg, ds, cg in variants():
        k = n_params(dg, ds, cg)
        if len(pts) < k + 3:
            continue
        A, S = fit(pts, dg, ds, cg)
        A, S = rounded(A), rounded(S)
        e = errors(A, S, pts)
        cv_rms, cv_max = cross_val(pts, groups, dg, ds, cg)
        notes = sane(A, S)
        results.append((cv_rms, k, name, ds, A, S, e, cv_max, notes, cg))
        print(f"{name:<9} slip deg {ds}  {k:3d}   {np.sqrt(np.mean(e**2)):6.3f}   {cv_rms:7.3f} / {cv_max:5.2f}"
              + ("   ! " + "; ".join(notes) if notes else ""))
    ok = [r for r in results if not r[8]]
    if not ok:
        sys.exit("\nevery variant has a problem (see !); more valve openings / pressures needed")
    best_cv = min(r[0] for r in ok)
    # simplest variant within 5 % of the best cross-validated error
    pick = sorted([r for r in ok if r[0] <= best_cv * 1.05], key=lambda r: (r[1], r[0]))[0]
    best_const = min([r for r in ok if r[9]], key=lambda r: r[0], default=None)

    def report(r, title):
        cv_rms, k, name, ds, A, S, e, cv_max, notes, cg = r
        print(f"\n== {title}: {name}, slip degree {ds}  (cross-validated RMS {cv_rms:.3f}, max {cv_max:.2f} ml/s)")
        print(f"   Pump Flow Coefficients  {fmt(A)}")
        print(f"   Pump Slip Coefficients  {fmt(S)}")
        g_new = lambda P: poly(A, P) + max(0.0, poly(S, P))
        g_old = lambda P: poly(cur_A, P) + max(0.0, poly(cur_S, P))
        print("   G(P) = d(flow)/d(power), ml/s per 100 %:  " + "  ".join(f"{P:.0f} bar {g_new(P):.1f}" for P in (0, 3, 6, 9)))
        conv = a.convergence * g_new(9.0) / g_old(9.0)
        print(f"   Convergence gain that keeps today's pressure-loop gain at 9 bar: {conv:.3g} (now {a.convergence})"
              + ("" if cg else "; with G not constant the loop gain then varies with pressure"))
        return A, S

    cur_A = [float(x) for x in a.current_flow.split(",")]
    cur_S = [float(x) for x in a.current_slip.split(",")]
    A, S = report(pick, "Pick (simplest within 5 % of the best cross-validation)")
    if best_const is not None and best_const is not pick:
        report(best_const, "Same structure as the current tune (G constant)")

    e_new = errors(A, S, pts)
    e_cur = errors(cur_A, cur_S, pts)
    print(f"\nper point (model as the firmware computes it; new = the pick above, current = {a.current_flow} / {a.current_slip})")
    print(f"  {'step':<48} {'power':>6} {'bar':>6} {'meter':>6} {'new':>6} {'err':>6} {'current':>8} {'err':>6} {'cup g/s':>8}")
    for p, en, ec in sorted(zip(pts, e_new, e_cur), key=lambda x: (x[0]["P"], x[0]["d"])):
        print(f"  {p['label']:<48} {p['d']*100:5.1f}% {p['P']:6.2f} {p['Q']:6.2f} {p['Q']+en:6.2f} {en:+6.2f} "
              f"{p['Q']+ec:8.2f} {ec:+6.2f} {p['cup']:8.2f}")
    print(f"\n  new:     RMS {np.sqrt(np.mean(e_new**2)):.3f}  max {np.max(np.abs(e_new)):.2f} ml/s")
    print(f"  current: RMS {np.sqrt(np.mean(e_cur**2)):.3f}  max {np.max(np.abs(e_cur)):.2f} ml/s")
    stalls = [(l, p) for l, p, w in rejected if w.startswith("flow ")]
    if stalls:
        d = np.array([p["d"] for _, p in stalls])
        P = np.array([p["P"] for _, p in stalls])
        qm = firmware_q(A, S, d, P)
        print("  stall check (measured < q-min): " + ", ".join(
            f"{p['d']*100:.0f}% {p['P']:.1f} bar meas {p['Q']:.2f} model {m:.2f}" for (_, p), m in zip(stalls, qm)))

    # repeatability: points from different logs at nearly the same power and pressure
    rep = []
    for i, j in itertools.combinations(range(len(pts)), 2):
        p, q = pts[i], pts[j]
        if p["file"] != q["file"] and abs(p["d"] - q["d"]) < 0.005 and abs(p["P"] - q["P"]) < 0.15:
            rep.append(abs(p["Q"] - q["Q"]))
    if rep:
        print(f"\nrepeatability (same power, pressure within 0.15 bar, different logs): {len(rep)} pairs, "
              f"median {np.median(rep):.3f}, max {np.max(rep):.3f} ml/s - no model gets below this")

    if a.csv_out:
        with open(a.csv_out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["file", "t0", "t1", "power_pct", "pressure_bar", "meter_ml_s", "pulses", "cup_g_s", "new_ml_s", "current_ml_s"])
            for p, en, ec in zip(pts, e_new, e_cur):
                w.writerow([p["file"], f"{p['t0']:.1f}", f"{p['t1']:.1f}", f"{p['d']*100:.2f}", f"{p['P']:.3f}", f"{p['Q']:.3f}",
                            p["pulses"], f"{p['cup']:.3f}", f"{p['Q']+en:.3f}", f"{p['Q']+ec:.3f}"])
        print(f"\npoints written to {a.csv_out}")


if __name__ == "__main__":
    main()
