"""Static load model of the two-leg robot (option D): legs of fixed length L, powered hinge angle phi,
clamps on one vertical rail (climb) or two horizontal rails (travel).  All values are ESTIMATES - edit below.

    pixi run -e sim python sim/legs.py            # table
    pixi run -e sim python sim/legs.py --payload 2 --phi 50

Frame: x along the wall (right), y out from the rail toward the robot (stand-off), z up.
Rail plane is y = 0.  The body is held by two ball-jointed leg ends P_A, P_B; the clamps carry only forces
(couples are made BY the two legs, spaced apart).
"""
import argparse, math
import numpy as np

G = 9.81
M_BODY = 1.3        # kg  chassis + 2 leg drives + electronics (sits at the leg ends)
M_TRAY = 1.5        # kg  tray + arm + basket (empty parts)
X_TRAY = 0.23       # m   tray centre sideways from the rail line (basket bay is beside the rail)
Y_TRAY = 0.10       # m   tray centre beyond the leg ends (stand-off)
L = 0.20            # m   fixed leg length
D = 0.30            # m   distance between the two leg ends on the body = horizontal-rail spacing
R_PIN = 0.0105      # m   pinion pitch radius (14T, module 1.5)
SHARE = 0.5         # share of the weight carried by the lower leg's drive
B_ALONG = 0.105     # m   clamp wheel base ALONG the rail (2 plates in tandem, as in v2)
B_ACROSS = 0.040    # m   clamp wheel stance ACROSS the rail (v2: 2 x 19.8 mm); try 0.12 for a wide clamp
Y_H = 0.0           # m   clamp hinge stand-off from the rail centre (v3: 33 mm)


def solve(payload, phi_deg, mode):
    """returns dict of forces (N) and hinge torques (N*m).  mode: 'climb' | 'travel'"""
    phi = math.radians(phi_deg)
    W_body, W_tray = M_BODY * G, (M_TRAY + payload) * G
    if mode == "climb":      # one vertical rail, legs in line; leg plane = y-z, hinge axis = x
        ca, cb = np.array([0, 0, 0.0]), np.array([0, 0, D + 2 * L * math.cos(phi)])
        pa, pb = ca + [0, L * math.sin(phi), L * math.cos(phi)], cb + [0, L * math.sin(phi), -L * math.cos(phi)]
        axis = np.array([1.0, 0, 0])
    else:                    # two horizontal rails D apart; leg plane = x-y, hinge axis = z
        ca, cb = np.array([0, 0, 0.0]), np.array([0, 0, D])
        pa, pb = ca + [L * math.cos(phi), L * math.sin(phi), 0], cb + [L * math.cos(phi), L * math.sin(phi), 0]
        axis = np.array([0, 0, 1.0])
    ys = pa[1]
    masses = [(W_body, np.array([0.0, ys, (pa[2] + pb[2]) / 2])),
              (W_tray, np.array([X_TRAY, ys + Y_TRAY, (pa[2] + pb[2]) / 2]))]
    W = sum(m for m, _ in masses)
    C = sum(m * p for m, p in masses) / W
    # unknown F_A, F_B (force of each leg ON the body).  equilibrium + weight split on the vertical axis
    cross = lambda r: np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
    A = np.zeros((7, 6)); b = np.zeros(7)
    A[0:3, 0:3] = np.eye(3); A[0:3, 3:6] = np.eye(3); b[0:3] = [0, 0, W]
    A[3:6, 0:3] = cross(pa - C); A[3:6, 3:6] = cross(pb - C)
    A[6, 2] = 1; b[6] = SHARE * W                                    # lower drive carries SHARE of the weight
    F = np.linalg.lstsq(A, b, rcond=None)[0]
    FA, FB = F[:3], F[3:]
    out = dict(W=W, x_c=C[0], y_c=C[1], FA=FA, FB=FB)
    # hinge torque: force the body applies to the leg end, moment about the hinge axis at the clamp
    out["hinge_A"] = abs(np.dot(np.cross(pa - ca, -FA), axis))
    out["hinge_B"] = abs(np.dot(np.cross(pb - cb, -FB), axis))
    out["span"] = float(np.linalg.norm(cb - ca))
    # the clamp receives the leg-end force AND the moment r x F (leg ends stand off from the rail)
    ia = 2 if mode == "climb" else 0                                 # rail axis index
    for k, (p_, c_, f_) in {"A": (pa, ca, FA), "B": (pb, cb, FB)}.items():
        M = np.cross(p_ - c_, f_)
        other = [abs(M[i]) for i in range(3) if i != ia]
        out["M_" + k] = M
        out["wheel_" + k] = (abs(M[ia]) / B_ACROSS, max(other) / B_ALONG)    # (across-rail pair, along-rail pair)
    return out


def solve_v3(payload, phi_deg, mode):
    """v3: ONE rail, legs in line along it, tray CENTRED on the rail line (X_TRAY = 0) and Y_TRAY beyond the leg ends.
    climb: gravity along the rail.  travel: rail horizontal, gravity across the rail (in the rack plane).
    Leg ends are treated as ball joints for the forces; the moment about the rail axis (travel: weight x stand-off)
    cannot pass through ball joints, so it is added as torsion, half per clamp (legs + hinges carry it)."""
    phi = math.radians(phi_deg)
    W_body, W_tray = M_BODY * G, (M_TRAY + payload) * G
    ca, cb = np.array([0, Y_H, 0.0]), np.array([0, Y_H, D + 2 * L * math.cos(phi)])
    pa, pb = ca + [0, L * math.sin(phi), L * math.cos(phi)], cb + [0, L * math.sin(phi), -L * math.cos(phi)]
    ys, zm = pa[1], (pa[2] + pb[2]) / 2
    g = np.array([0, 0, 1.0]) if mode == "climb" else np.array([1.0, 0, 0])    # direction of the support force
    masses = [(W_body, np.array([0.0, ys, zm])), (W_tray, np.array([X_TRAY, ys + Y_TRAY, zm]))]
    W = sum(m for m, _ in masses)
    C = sum(m * p for m, p in masses) / W
    cross = lambda r: np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
    A = np.zeros((6, 6)); b = np.zeros(6)
    A[0:3, 0:3] = np.eye(3); A[0:3, 3:6] = np.eye(3); b[0:3] = W * g
    Pm = (pa + pb) / 2                                                   # moments about a point on the leg-end line
    A[3:5, 0:3] = cross(pa - Pm)[0:2]; A[3:5, 3:6] = cross(pb - Pm)[0:2]
    b[3:5] = -np.cross(C - Pm, W * g)[0:2] * -1
    A[5, 2 if mode == "climb" else 0] = 1; b[5] = SHARE * W              # lower / left drive carries SHARE
    F = np.linalg.lstsq(A, b, rcond=None)[0]
    FA, FB = F[:3], F[3:]
    torsion = abs(np.cross(C - np.array([0, 0, zm]), W * g)[2]) / 2 if mode == "travel" else 0.0   # about the rail axis, per clamp
    out = dict(W=W, FA=FA, FB=FB, span=float(np.linalg.norm(cb - ca)), torsion=torsion)
    for k, (p_, c_, f_) in {"A": (pa, ca, FA), "B": (pb, cb, FB)}.items():
        M = np.cross(p_ - np.array([0, 0, c_[2]]), f_)                  # about the clamp's wheel centre (on the rail)
        out["hinge_" + k] = abs(np.dot(np.cross(p_ - c_, -f_), [1.0, 0, 0]))
        m_axis = torsion if mode == "travel" else abs(M[2])            # travel: the whole W x stand-off, half per clamp
        out["wheel_" + k] = (m_axis / B_ACROSS, max(abs(M[0]), abs(M[1])) / B_ALONG)
    return out


def report_v3(payload, phi):
    for mode in ("climb", "travel"):
        r = solve_v3(payload, phi, mode)
        wa = max(r["wheel_A"][0], r["wheel_B"][0]); wl = max(r["wheel_A"][1], r["wheel_B"][1])
        pin = (SHARE * r["W"] * R_PIN) if mode == "climb" else 0.0
        print(f"  {mode:6s} span {r['span']*1000:4.0f} mm | WHEEL load: across-rail {wa:5.0f} N (stance {B_ACROSS*1000:.0f} mm), "
              f"along-rail {wl:4.0f} N | leg hinge {max(r['hinge_A'], r['hinge_B']):4.2f} N*m | climb-pinion {pin:4.2f} N*m"
              f" | torsion on each clamp {r['torsion']:4.2f} N*m")


def baseline(payload):
    """current v2: one rail, wheels 40 mm apart across the rail (travel) and 105 mm along it (climb)"""
    W = (M_BODY + M_TRAY + payload) * G
    y = 0.12 + 0.0
    return dict(travel_wheel=W * y / 0.0396, climb_wheel=W * 0.23 / 0.105)       # v2 tray 230 mm beside the rail


def report(payload, phi):
    for mode in ("climb", "travel"):
        r = solve(payload, phi, mode)
        lat = lambda F: np.linalg.norm(F[:2]) if mode == "climb" else np.linalg.norm(F[[1, 2]])
        axial = abs(r["FA"][0]) if mode == "travel" else 0.0
        fy = max(abs(r["FA"][1]), abs(r["FB"][1]))
        fx = max(abs(r["FA"][0]), abs(r["FB"][0]))
        pin = (SHARE * r["W"] if mode == "climb" else 0.0) * R_PIN
        wa = max(r["wheel_A"][0], r["wheel_B"][0]); wl = max(r["wheel_A"][1], r["wheel_B"][1])
        print(f"  {mode:6s} span {r['span']*1000:4.0f} mm | WHEEL load: across-rail {wa:5.0f} N (stance {B_ACROSS*1000:.0f} mm), along-rail {wl:4.0f} N "
              f"| leg hinge {max(r['hinge_A'], r['hinge_B']):4.2f} N*m | climb-pinion {pin:4.2f} N*m"
              + (f" | drive axial (couple) {axial:4.1f} N each rail" if mode == "travel" else ""))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--payload", type=float, default=None); ap.add_argument("--phi", type=float, default=None)
    ap.add_argument("--stance", type=float, default=None, help="clamp wheel stance across the rail, m")
    ap.add_argument("--v3", action="store_true", help="use the dimensions of generator/params_v3.py")
    a = ap.parse_args()
    if a.v3:     # v3 = compact Squid layout: two horizontal rails D apart, fixed battery box near the rails (estimates)
        import os, sys
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "generator"))
        import params_v3 as P3
        D_ = P3.D_HUB / 1000
        outer = (P3.D_HUB + 2 * (P3.HUB_WHEEL_A + P3.WHEEL_R)) / 1000
        bx = -(P3.BAT_X[0] + P3.BAT_X[1]) / 2 / 1000
        by = (-50 - (P3.BAT_Y[0] + P3.BAT_Y[1]) / 2) / 1000
        m_tray = 1.0                                               # tray + arms + fingers (estimate, no crane / slide)
        print(f"v3 compact: hubs {D_*1000:.0f} mm apart (= horizontal rail spacing); battery {P3.BAT_MASS} kg fixed "
              f"{bx*1000:.0f} mm beside the column, {by*1000:.0f} mm in front of the rails")
        for p in ([a.payload] if a.payload is not None else [0.0, 0.5, 1.0, 2.0]):
            b = baseline(p)
            mx = 4.1 * m_tray / 1.5 + 2.35 * p - P3.BAT_MASS * G * bx
            my = (M_BODY + m_tray + p) * G * 0.11 + P3.BAT_MASS * G * by
            loads = dict(forward=my / D_, twist_wheels=abs(mx) / outer, twist_pinions=abs(mx) / D_)
            print(f"payload {p} kg: forward-tipping {loads['forward']:3.0f} N | twist on wheels {loads['twist_wheels']:3.0f} N | "
                  f"twist on pinions {loads['twist_pinions']:3.0f} N | worst {max(loads.values()):3.0f} N "
                  f"(v2 {b['climb_wheel']:.0f} N climbing, {b['travel_wheel']:.0f} N travelling)")
        raise SystemExit
    if a.stance: B_ACROSS = a.stance
    pls = [a.payload] if a.payload is not None else [0.0, 0.5, 1.0, 2.0]
    phis = [a.phi] if a.phi is not None else ([round(math.degrees(math.asin(40 / 150)), 1), 90] if a.v3 else [90, 60, 40])
    for p in pls:
        b = baseline(p)
        print(f"payload {p} kg   (current v2 single rail: wheel force climb {b['climb_wheel']:.0f} N, travel {b['travel_wheel']:.0f} N)")
        for ph in phis:
            print(f" leg angle phi={ph} deg")
            (report_v3 if a.v3 else report)(p, ph)
