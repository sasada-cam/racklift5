"""RackLift5 v3 in the MuJoCo viewer (kinematic: poses are commanded directly, no physics).
Compact Squid layout: two horizontal rails + one vertical rail per bay, small crossing turntables; the robot's two hubs
swivel 90 deg (one at a time) while the body never rotates.  Fixed battery box as counterweight, load cell under the tray,
2-stage telescopic arms (stacked bars, belt-doubled) with front and rear flip fingers that capture the basket; two servos
drive the whole picker (servo 1 the arms, servo 2 all four fingers) and every gear, pulley and belt run moves.

    pixi run -e sim sim                     # viewer (macOS uses mjpython) + task page in the browser
    pixi run -e sim sim-headless            # no window: run task 1, save snapshots
    pixi run -e sim sim-headless --task 5   # any task by number

Pre-defined tasks (each one continuous sequence, several joints moving together) are started from the task page
that opens in the browser (http://127.0.0.1:8765, see sim/remote.py) - that page is the only place they are listed.
A task number + Enter in the terminal, or keys 6/7/8/9 in the viewer (fetch next / operator takes / demo / reset), also work.
Sliders in the viewer (right panel -> Control) move the robot by hand while it is idle.
"""
import argparse, math, os, sys, time
from collections import deque
import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import model_v3 as M3
P3, A3 = M3.P3, M3.A3
MM = 1e-3
HALF_PI = math.pi / 2
SPEED = dict(x=250.0, z=120.0, h=90.0, arm=150.0, f=300.0, ry=150.0, lift=40.0, lock=60.0, jack=30.0, dsl=100.0)   # mm/s, deg/s
N_LEV = len(P3.LEVELS)
NB, RB = P3.N_BAYS_ALL, P3.ROW_BAYS
CROSS = [f"r{j}{k}" for j in range(NB) for k in range(2)]     # crossing piece angles per bay, k 0 lower 1 upper
_M = [0.4, 1.2, 0.8, 0.3, 1.6, 0.6, 0.2, 1.0, 1.4, 0.5, 0.9, 0.3, 1.1, 0.7, 1.3, 0.2, 1.5, 0.6, 1.0, 0.4]
MASS = {(b, l): _M[(7 * b + l) % len(_M)] for b in range(NB) for l in range(N_LEV)}    # payload per basket (kg, demo)
MASS[(NB, 0)] = 0.7                                                                       # the parcel the delivery robot brings
MAX_PAYLOAD = 2.0
G = 9.81


def speed_key(k):
    if k in ("cart", "ry"):
        return "ry"
    if k in ("h1", "h2") or k.startswith("r"):
        return "h"
    return "jack" if (k.startswith("j") and k[1:].isdigit()) else k


def cname(k):
    return f"crossing_{int(k[1:-1])+1}_{'lower' if k[-1] == '0' else 'upper'}"


class Sim:
    def __init__(self, speed=1.0):
        self.m = mujoco.MjModel.from_xml_path(M3.build())
        self.d = mujoco.MjData(self.m)
        self.speed = speed
        joints = ["x", "y", "z", "hub_upper", "hub_lower", "arm_mid", "arm_in", "pinion_upper", "pinion_lower"] + [f"cross{c[1:]}" for c in CROSS] \
            + ["cart", "cart_lock", "turn_lift", "turn_pinion"] + M3.TURNING \
            + [f"dock{r}_{p}" for r in range(P3.N_ROWS) for p in ("platform", "arm1", "arm2", "slider", "screw")]
        self.belt_geoms = {(sx, k): self.m.geom(M3.belt_name(sx, k)).id for _, sx, k, _ in A3.stage2_runs(P3.TIP0)}
        self.qa = {j: self.m.joint(j).qposadr[0] for j in joints}
        self.act = {n: self.m.actuator(n).id for n in
                    ["z", "hub_upper_swivel", "hub_lower_swivel", "servo1_arms", "servo2_fingers"] + [cname(c) for c in CROSS]
                    + [f"dock{r+1}_jack" for r in range(P3.N_ROWS)]}
        self.slots = [(0, 2), (RB + 2, 4), (1, 0), (2 * RB + 1, 3), (0, 4), (RB, 1)]
        self.ALL_SLOTS = [(b, l) for b in range(NB) for l in range(N_LEV)]
        self.tasks = {
            1: ("Fetch row 1 bay 1 shelf 3", lambda: self.fetch(0, 2)),
            2: ("Fetch row 3 bay 4 shelf 5 (far corner, top)", lambda: self.fetch(NB - 1, 4)),
            3: ("Fetch next basket", lambda: self.fetch(*self.next_slot()) if self.next_slot() else self.note("no baskets left")),
            4: ("Put the basket back on its shelf", self.put_back),
            5: ("Operator takes the basket", self.unload),
            6: ("Operator gives a basket back (onto the tray)", self.reload),
            7: ("Inspection tour (all 3 rows, by the transfer cart)", self.tour),
            8: ("Full demo (fetch, put back, fetch, hand-over)", self.demo),
            9: ("Park at home", self.park),
            10: ("Reset", None),
            11: ("Weight-balance demo (heavy basket, slow, battery on/off)", self.balance_demo),
            12: ("Fetch row 2 bay 2 shelf 2", lambda: self.fetch(RB + 1, 1)),
            13: ("Deliver row 1 bay 2 shelf 2 to dock 1 (jack lowers it for the home-delivery robot)", lambda: self.deliver(1, 1, 0)),
            14: ("Deliver row 3 bay 3 shelf 4 to dock 3 (far row, by the cart)", lambda: self.deliver(2 * RB + 2, 3, 2)),
            15: ("Home-delivery robot takes the baskets from the docks", self.delivery_takes),
            16: ("Home-delivery robot brings a parcel to dock 2; robot stores it in an empty slot", lambda: self.delivery_brings(1)),
            17: ("Rearrange: move row 1 bay 1 shelf 3 to the empty row 2 bay 4 shelf 5", lambda: self.rearrange((0, 2), (RB + 3, 4))),
            18: ("Rearrange: move row 3 bay 3 shelf 2 to the empty row 1 bay 4 shelf 2", lambda: self.rearrange((2 * RB + 2, 1), (3, 1))),
        }
        self.snaps = []
        self.log = deque(maxlen=8)
        self.bat_on, self.show_balance, self.slowmo = True, True, 1.0
        self.reset()

    # ------------------------------------------------------------------ state -> qpos
    def reset(self):
        mujoco.mj_resetData(self.m, self.d)
        self.x_home = A3.bay_rail_x(0) + 130.0     # home: row 1, tray in front of its first bay (operator hand-over)
        self.row = 0
        self.cmd = dict(x=self.x_home, z=P3.CZ_HOME, h1=-90.0, h2=-90.0, arm=P3.TIP0, f=0.0, ry=0.0, cart=0.0,
                        lift=P3.TURN_LIFT, lock=0.0, dsl=P3.BASKET_Y0, **{c: -90.0 for c in CROSS},
                        **{f"j{r}": P3.JACK_DOWN for r in range(P3.N_ROWS)})
        # baskets: id = the slot the basket started in; state: shelf (at self.slot[id]) / held / tray / dock / dslide
        # (sliding on or off a dock's raised floor) / out (operator) / away (home-delivery robot)
        self.basket = {k: "shelf" for k in P3.BASKET_IDS if k[0] < NB}
        self.basket[(NB, 0)] = "away"                  # a parcel the home-delivery robot can bring (task 16)
        self.slot = {k: k for k in self.basket if k[0] < NB}
        self.home = dict(self.slot)                    # the shelf slot a basket was last fetched from
        self.reserved = set()                          # slots a basket is being pushed into
        self.ctx, self.dock_of = {}, {}
        self.away_pos = {(NB, 0): (P3.X_CART + 700.0, 80.0, 0.0)}
        self.out_pos, self.out_order = {}, []
        self.hold = {}                                   # basket -> near-wall y while the fingers hold it
        self.gen = None; self.move = None; self.queue = deque(); self.t = 0.0
        self.task_name, self.step_name = "idle", ""
        self.apply()

    @property
    def payload(self):
        return sum(MASS[k] for k, st in self.basket.items() if st == "tray")

    def S(self):
        c = self.cmd
        return dict(cx=c["x"], cz=c["z"], ry=c["ry"])

    def robot_M(self):
        return A3.robot_world(self.S())

    def apply(self):
        c, q, d = self.cmd, self.qa, self.d
        r = math.radians
        half = (c["arm"] - P3.TIP0) / 2 * MM
        Rw = self.robot_M()
        vals = dict(x=Rw[0, 3] * MM, y=Rw[1, 3] * MM, z=Rw[2, 3] * MM, cart=c["cart"] * MM, turn_lift=c["lift"] * MM, cart_lock=c["lock"] * MM,
                    turn_pinion=r(A3.turn_pinion_angle(c["cart"])), hub_upper=r(c["h1"]), hub_lower=r(c["h2"]), arm_mid=half, arm_in=half,
                    **{f"cross{k[1:]}": r(c[k]) for k in CROSS})
        # every gear / pulley turns with the motion it drives (see A3.mech_angles): climb pinions roll along the rack;
        # servo 1 -> step-up belt -> cross shaft -> stage-1 belts move the middle bars; the stage-2 pulleys turn as the
        # inner bars run at twice the speed; servo 2 turns the left finger rod through a gear pair (opposite way) and
        # the right one through a belt (same way)
        roll = -(c["x"] + c["z"]) / P3.PIN_R
        vals["pinion_upper"], vals["pinion_lower"] = roll, roll
        g = A3.mech_angles(c["arm"], c["f"])
        for j, key in (("arm_drive", "drive"), ("arm_idler", "drive"), ("servo1", "servo1"), ("servo2", "servo2"),
                       ("dpul_ext_l", "dpul"), ("dpul_ext_r", "dpul"), ("dpul_ret_l", "dpul"), ("dpul_ret_r", "dpul")):
            vals[j] = r(g[key])
        for sd in ("l", "r"):
            for j in ("fshaft", "ftube", "finger"):
                vals[f"{j}_{sd}"] = r(g[f"finger_{sd}"])
        for _, sx, k, (x0, x1, y0, y1, z0, z1) in A3.stage2_runs(c["arm"]):     # belt runs stretch / shrink
            gi = self.belt_geoms[(sx, k)]
            self.m.geom_pos[gi] = np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]) * MM
            self.m.geom_size[gi] = np.array([(x1 - x0) / 2, max(y1 - y0, 0.2) / 2, (z1 - z0) / 2]) * MM
        for rr in range(P3.N_ROWS):        # the docks' scissor jacks: arms, slider and lead screw follow the platform height
            zt = c[f"j{rr}"]
            phi, sx = A3.jack_angles(zt)
            vals.update({f"dock{rr}_platform": zt * MM, f"dock{rr}_arm1": -phi, f"dock{rr}_arm2": phi,
                         f"dock{rr}_slider": sx * MM, f"dock{rr}_screw": r(A3.jack_screw_angle(zt))})
        for k, v in vals.items():
            d.qpos[q[k]] = v
        for k, st in self.basket.items():
            if st == "shelf":
                M = A3.basket_M(*self.slot[k])
            elif st == "held":                 # captured between front and rear fingers (2 mm play): pulled or pushed
                tip = c["arm"]
                y0 = min(max(self.hold[k], tip - P3.REAR_OFF), tip - P3.FINGER_T - P3.BASKET_D)
                self.hold[k] = y0
                kind, where = self.ctx[k]
                M = A3.basket_M(*where, y0) if kind == "shelf" else A3.dock_basket_M(where, c[f"j{where}"], y0)
            elif st == "tray":
                M = Rw @ A3.MT(P3.BXR, P3.BASKET_TRAY_Y0, P3.T_V3)
            elif st == "dock":
                rr = self.dock_of[k]
                M = A3.dock_basket_M(rr, c[f"j{rr}"], P3.BASKET_Y0)
            elif st == "dslide":               # sliding over the raised floor to / from the home-delivery robot
                rr = self.dock_of[k]
                M = A3.dock_basket_M(rr, c[f"j{rr}"], c["dsl"])
            elif st == "away":
                M = A3.MT(*self.away_pos[k])
            else:
                M = A3.MT(*self.out_pos[k])
            yaw = math.atan2(M[1, 0], M[0, 0])
            a = self.m.joint(f"bj_{k[0]}_{k[1]}").qposadr[0]
            d.qpos[a:a + 7] = [M[0, 3] * MM, M[1, 3] * MM, M[2, 3] * MM, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        u, a = d.ctrl, self.act
        u[a["z"]] = c["z"] * MM
        for rr in range(P3.N_ROWS):
            u[a[f"dock{rr+1}_jack"]] = c[f"j{rr}"] * MM
        u[a["hub_upper_swivel"]], u[a["hub_lower_swivel"]] = r(c["h1"]), r(c["h2"])
        u[a["servo1_arms"]] = half
        u[a["servo2_fingers"]] = r(g["servo2"])
        for k in CROSS:
            u[a[cname(k)]] = r(c[k])
        mujoco.mj_forward(self.m, d)

    def from_sliders(self):
        u, a, c = self.d.ctrl, self.act, self.cmd
        g = math.degrees
        c.update(z=u[a["z"]] / MM, h1=g(u[a["hub_upper_swivel"]]), h2=g(u[a["hub_lower_swivel"]]),
                 arm=P3.TIP0 + 2 * u[a["servo1_arms"]] / MM, f=-g(u[a["servo2_fingers"]]))
        for k in CROSS:
            c[k] = g(u[a[cname(k)]])
        for rr in range(P3.N_ROWS):
            c[f"j{rr}"] = u[a[f"dock{rr+1}_jack"]] / MM

    # ------------------------------------------------------------------ building blocks
    def mv(self, step, **t):
        return ("move", step, t)

    def say(self, s):
        print(f"[{self.t:6.1f}s] {s}", flush=True)
        self.log.append(f"[{self.t:5.0f}s] {s}")     # also shown on the task page

    def note(self, s):
        self.say(s)
        return iter(())

    def weigh(self):
        m = self.payload
        self.say(f"load cell: {m:.1f} kg on the tray" + (" - OVERLOAD" if m > MAX_PAYLOAD else ""))

    def to_flat(self):
        c = self.cmd
        if c["h1"] > -45 or c["h2"] > -45:
            b = self.row * RB + int(np.argmin([abs(c["x"] - A3.bay_rail_x(j)) for j in range(RB)]))
            yield from self.leave_bay(b)

    def goto_row(self, r):
        """to row r by the transfer cart at the rows' right ends: drive onto the cart, drop the pinion into the fixed
        transfer rack, push robot + cart across the aisles, lift the pinion, drive off into the row"""
        if self.row == r:
            return
        yield self.mv("drive off the row onto the transfer cart's rail pieces (arms stowed)", x=P3.X_CART, arm=P3.TIP0, f=0.0)
        yield self.mv("SG90 lowers the robot's pinion into the transfer rack; the robot's hub presses the cart's lock lever: "
                      "the lock pin leaves the row's socket", lift=0.0, lock=22.0)
        yield ("snap", "t1_on_cart")
        yield self.mv(f"the robot's pinion pushes itself + the cart along the side track to row {r+1} "
                      f"(hub motors hold it on the cart)", ry=r * P3.ROW_P, cart=r * P3.ROW_P)
        yield ("snap", "t2_at_new_row")
        yield self.mv("lock pin springs into this row's socket (cart held in line with the row's rails); pinion lifts clear",
                      lift=P3.TURN_LIFT, lock=0.0)
        yield ("call", lambda: setattr(self, "row", r))
        self.row = r                              # plan ahead (the call above does it again when it runs)

    def go_home(self):
        yield from self.goto_row(0)
        yield self.mv("travel home (row 1, operator side)", x=self.x_home)

    def enter_bay(self, b, z_target):
        """from the two horizontal rails onto bay b's vertical rail: one hub swivels at a time (the other stays locked)"""
        yield from self.goto_row(A3.bay_row(b))
        yield self.mv(f"travel to bay {b % RB + 1}'s rail crossings (row {A3.bay_row(b) + 1})", x=A3.bay_rail_x(b))
        yield self.mv("lower hub swivels 90 deg with its crossing piece (upper hub stays locked on its rail)",
                      h2=0.0, **{f"r{b}0": 0.0})
        yield ("snap", "1_swivel")
        yield self.mv("upper hub swivels 90 deg with its crossing piece (lower hub locked)", h1=0.0, **{f"r{b}1": 0.0})
        if z_target > P3.CZ_HOME + 1:
            yield self.mv("climb the single vertical rail (both hub motors drive; the body never tilts)", z=z_target)

    def leave_bay(self, b):
        yield self.mv("climb down to the crossings, arms stowed", z=P3.CZ_HOME, arm=P3.TIP0, f=0.0, x=A3.bay_rail_x(b))
        yield self.mv("upper hub swivels back to its horizontal rail", h1=-90.0, **{f"r{b}1": -90.0})
        yield self.mv("lower hub swivels back to its horizontal rail", h2=-90.0, **{f"r{b}0": -90.0})

    # ------------------------------------------------------------------ baskets, slots and docks
    @staticmethod
    def where(b):
        return f"row {b // RB + 1} bay {b % RB + 1}"

    def label(self, k):
        return "the parcel" if k[0] >= NB else f"the basket from {self.where(k[0])} shelf {k[1]+1}"

    def id_at(self, b, l):
        return next((k for k, st in self.basket.items() if st == "shelf" and self.slot[k] == (b, l)), None)

    def empty_slots(self):
        full = {self.slot[k] for k, st in self.basket.items() if st == "shelf"} | self.reserved
        return [s for s in self.ALL_SLOTS if s not in full]

    def pick_empty(self, prefer=None):
        """prefer this slot if it is free, else the empty slot nearest to the robot's row"""
        e = self.empty_slots()
        if prefer in e:
            return prefer
        e.sort(key=lambda s: (abs(A3.bay_row(s[0]) - self.row), s[0], s[1]))
        return e[0] if e else None

    def on_tray(self):
        return next((k for k, st in self.basket.items() if st == "tray"), None)

    def dock_basket(self, r):
        return next((k for k, st in self.basket.items() if st in ("dock", "dslide") and self.dock_of.get(k) == r), None)

    # ------------------------------------------------------------------ tasks
    # Every task makes its own way: a fetch with a basket still on the tray first puts that one back, a hand-back first
    # parks the robot at home, ... so tasks can be clicked one after another (they queue) in any order.
    def clear_tray(self):
        """put a basket that is still on the tray back on a shelf (no trip home in between)"""
        k = self.on_tray()
        if k is not None:
            yield ("call", lambda: self.say(f"tray is full: putting {self.label(k)} back first"))
            yield from self._put_back(k, home=False)

    def fetch(self, b, l):
        k = self.id_at(b, l)
        if k is None:
            return self.note(f"{self.where(b)} shelf {l+1} is empty")
        return self._fetch(k)

    def _fetch(self, k, home=True):
        yield from self.clear_tray()
        b, l = self.slot[k]
        self.home[k] = (b, l)
        yield from self.to_flat()
        yield from self.enter_bay(b, P3.cz_for_level(l))
        yield ("snap", "2_at_shelf")
        yield from self._pull_here(k, ("shelf", (b, l)))
        yield ("call", self.weigh)
        yield from self.leave_bay(b)
        if home:
            yield from self.go_home()
            yield ("call", lambda: self.say("delivered - task 5: operator takes it, task 4: put it back"))

    def put_back(self):
        k = self.on_tray()
        if k is None:
            return self.note("nothing on the tray - run a fetch, or task 6 to hand a basket back")
        return self._put_back(k)

    def _put_back(self, k, dest=None, home=True):
        """store the tray basket k on a shelf: dest if it is free, else the shelf it came from, else the nearest empty slot"""
        dest = self.pick_empty(dest if dest is not None else self.home.get(k))
        if dest is None:
            yield ("call", lambda: self.say("no empty slot on any shelf - the basket stays on the tray"))
            return
        b, l = dest
        self.reserved.add(dest)
        yield from self.to_flat()
        yield from self.enter_bay(b, P3.cz_for_level(l))
        yield from self._put_here(k, ("shelf", dest))
        yield ("snap", "7_pushed_back")
        yield ("call", self.weigh)
        yield from self.leave_bay(b)
        if home:
            yield from self.go_home()
        yield ("call", lambda: self.say(f"{self.label(k)} is now on {self.where(b)} shelf {l+1}"))

    def rearrange(self, src, dst):
        k = self.id_at(*src)
        if k is None:
            return self.note(f"{self.where(src[0])} shelf {src[1]+1} is empty - nothing to move")
        if dst not in self.empty_slots():
            return self.note(f"{self.where(dst[0])} shelf {dst[1]+1} is not empty")
        return self._rearrange(k, dst)

    def _rearrange(self, k, dst):
        yield from self._fetch(k, home=False)
        yield from self._put_back(k, dst)

    # ---- docking stations: one per row, at the end of the row, a motorized scissor jack under a platform
    def to_dock(self, r):
        """drive to dock r (via the cart if it is in another row); its jack rises to tray height meanwhile"""
        yield from self.goto_row(r)
        yield self.mv(f"drive to dock {r+1} at the end of row {r+1}; its jack rises to tray height meanwhile",
                      x=P3.DOCK_X_C, arm=P3.TIP0, f=0.0, **{f"j{r}": P3.JACK_UP})

    def deliver(self, b, l, r):
        k = self.id_at(b, l)
        if k is None:
            return self.note(f"{self.where(b)} shelf {l+1} is empty")
        if self.dock_basket(r) is not None:
            return self.note(f"dock {r+1} is occupied - task 15: the delivery robot takes it first")
        return self._deliver(k, r)

    def _deliver(self, k, r):
        yield from self._fetch(k, home=False)
        yield from self.to_dock(r)
        yield ("snap", "d1_at_dock")
        yield from self._put_here(k, ("dock", r))
        yield ("snap", "d2_on_platform")
        yield self.mv(f"dock {r+1}: the jack lowers the basket from tray height to the raised floor", **{f"j{r}": P3.JACK_DOWN})
        yield ("snap", "d3_lowered")
        yield ("call", lambda: self.say(f"{self.label(k)} waits on dock {r+1} for the home-delivery robot (task 15)"))
        yield from self.go_home()

    def delivery_takes(self):
        ks = [(k, self.dock_of[k]) for k, st in self.basket.items() if st == "dock"]
        if not ks:
            return self.note("no basket is waiting on a dock - run task 13 or 14 first")
        return self._delivery_takes(ks)

    def _delivery_takes(self, ks):
        for k, r in ks:
            if self.cmd[f"j{r}"] > P3.JACK_DOWN + 1:
                yield self.mv(f"dock {r+1}: the jack lowers the basket", **{f"j{r}": P3.JACK_DOWN})

            def start(k=k):
                self.basket[k] = "dslide"
                self.cmd["dsl"] = P3.BASKET_Y0
            yield ("call", start)
            yield self.mv(f"the home-delivery robot takes the basket off dock {r+1}: it slides back over the raised floor",
                          dsl=P3.DECK_Y[1] - P3.BASKET_D - 2.0)

            def gone(k=k, r=r):
                n = sum(1 for st in self.basket.values() if st == "away")
                self.away_pos[k] = (P3.X_CART + 700.0, 80.0 + 220.0 * n, 0.0)
                self.basket[k] = "away"
                self.dock_of.pop(k, None)
                self.cmd["dsl"] = P3.BASKET_Y0
                self.say(f"{self.label(k)} left with the home-delivery robot")
            yield ("call", gone)

    def delivery_brings(self, r):
        away = sorted((k for k, st in self.basket.items() if st == "away"), key=lambda k: -k[0])   # the spare parcel first
        k = away[0] if away else None
        if k is None:
            return self.note("no basket is out for delivery")
        if self.dock_basket(r) is not None:
            return self.note(f"dock {r+1} is occupied - task 15: the delivery robot takes it first")
        return self._delivery_brings(k, r)

    def _delivery_brings(self, k, r):
        yield from self.clear_tray()

        def arrive():
            self.basket[k] = "dslide"; self.dock_of[k] = r
            self.cmd["dsl"] = P3.DECK_Y[1] - P3.BASKET_D - 2.0
        yield ("call", arrive)
        yield self.mv(f"the home-delivery robot slides {self.label(k)} onto dock {r+1}'s lowered platform", dsl=P3.BASKET_Y0)
        yield ("call", lambda: self.basket.__setitem__(k, "dock"))
        yield from self.to_flat()
        yield from self.to_dock(r)
        yield ("snap", "d4_parcel_up")
        yield from self._pull_here(k, ("dock", r))
        yield self.mv(f"dock {r+1}: the jack lowers (empty)", **{f"j{r}": P3.JACK_DOWN})
        yield ("call", lambda: self.say(f"{self.label(k)} is on the tray - looking for an empty slot"))
        yield from self._put_back(k, None, home=True)

    def unload(self):
        k = self.on_tray()
        if k is not None:
            self.out_pos[k] = (self.x_home + 300 * len(self.out_pos), -900, 2)
            self.out_order.append(k)
            self.basket[k] = "out"
            self.say(f"{self.label(k)} taken by the operator")
            self.apply()
            return iter(())
        self.say("nothing on the tray")
        return iter(())

    def reload(self):
        if not any(st == "out" for st in self.basket.values()):
            return self.note("the operator has no basket")
        return self._reload()

    def _reload(self):
        yield from self.clear_tray()                  # tray must be empty to take a basket
        yield from self.park()                        # the operator hands it over at home

        def give():
            k = next((k for k in reversed(self.out_order) if self.basket[k] == "out"), None)
            if k is None:
                return self.say("the operator has no basket")
            self.basket[k] = "tray"; self.out_order.remove(k); self.out_pos.pop(k, None)
            self.say(f"operator put {self.label(k)} on the tray - task 4 puts it on a shelf")
            self.weigh(); self.apply()
        yield ("call", give)

    def tour(self):
        yield from self.to_flat()
        for b in [RB - 1, 2 * RB - 1, 3 * RB - 2][:P3.N_ROWS]:
            yield from self.enter_bay(b, P3.cz_for_level(N_LEV - 1))
            yield self.mv(f"{self.where(b)}: arms out along the top-shelf basket (fingers up)", arm=P3.TIP_SHELF)
            yield self.mv("arms back", arm=P3.TIP0)
            yield from self.leave_bay(b)
        yield from self.go_home()

    def demo(self):
        s = self.next_slot()
        if s is None:
            return self.note("no baskets on the shelves")
        return self._demo(self.id_at(*s))

    def _demo(self, k):
        yield from self._fetch(k)
        yield from self._put_back(k)
        k2 = next((self.id_at(*x) for x in self.slots + self.ALL_SLOTS if self.id_at(*x) not in (None, k)), None)
        if k2 is not None:
            yield from self._fetch(k2)
            yield ("call", lambda: self.unload())

    def balance_demo(self):
        k = max((k for k, st in self.basket.items() if st == "shelf"), key=lambda k: MASS[k], default=None)
        if k is None:
            return self.note("no baskets on the shelves")
        return self._balance_demo(k)

    def _balance_demo(self, k):
        def bl(msg):
            def f():
                x = self.balance()
                self.say(f"{msg}: twist {x['mx']:+.2f} N*m ({abs(x['f_twist']):.0f} N/hub), forward {x['my']:.2f} N*m "
                         f"({abs(x['f_fwd']):.0f} N/hub)")
            return ("call", f)
        yield ("call", lambda: setattr(self, "show_balance", True))
        yield from self.clear_tray()
        b, l = self.slot[k]
        self.home[k] = (b, l)
        yield from self.to_flat()
        yield from self.enter_bay(b, P3.cz_for_level(l))
        yield bl("empty tray")
        yield from self._pull_here(k, ("shelf", (b, l)), f"SLOW: {MASS[k]:.1f} kg basket slides onto the tray - watch the arrows grow")
        yield bl("loaded, battery on")
        yield ("snap", "8_balance_battery_on")
        yield ("pause", "loaded: the battery box on the left balances part of the tray's twist", 3.0)
        yield ("call", lambda: setattr(self, "bat_on", False))
        yield bl("loaded, battery ignored")
        yield ("snap", "9_balance_battery_off")
        yield ("pause", "battery ignored: the twist (blue arrows) gets bigger", 3.0)
        yield ("call", lambda: setattr(self, "bat_on", True))
        yield ("pause", "battery back on", 1.5)
        self.reserved.add((b, l))
        yield from self._put_here(k, ("shelf", (b, l)))
        yield from self.leave_bay(b)
        yield from self.go_home()

    def _grab(self, k, y0, ctx):
        def f():
            self.hold[k] = y0
            self.ctx[k] = ctx
            self.basket[k] = "held"
        return ("call", f)

    def _pull_here(self, k, src, slow_note=None):
        """pull basket k from its shelf slot / dock platform onto the tray (the arms are beside it, fingers captured)"""
        yield self.mv("servo 1: arms slide out along the basket's sides (fingers up)", arm=P3.TIP_SHELF)
        yield ("snap", "3_arms_out")
        yield self.mv("servo 2: all four fingers swing in - front pair behind the far wall, rear pair before the near wall",
                      f=90.0)
        yield ("snap", "4_captured")
        yield self._grab(k, P3.BASKET_Y0, src)
        if slow_note:
            yield ("call", lambda: setattr(self, "slowmo", 0.25))
        yield ("slow", self.mv(slow_note or "servo 1 back: the 2:1 belts pull the basket onto the tray", arm=P3.TIP_PULLED))
        yield ("call", lambda: setattr(self, "slowmo", 1.0))

        def landed():
            self.basket[k] = "tray"
            if src[0] == "dock":
                self.dock_of.pop(k, None)
        yield ("call", landed)
        yield ("snap", "5_pulled")
        yield self.mv("servo 2: fingers swing up", f=0.0)
        yield self.mv("arms stowed", arm=P3.TIP0)

    def _put_here(self, k, dst):
        """push the tray basket k onto a shelf slot / dock platform (the robot stands in front of it)"""
        what = "platform" if dst[0] == "dock" else "shelf"
        yield self.mv("servo 1: arm tips to the basket on the tray", arm=P3.TIP_TRAY)
        yield self.mv("servo 2: fingers swing in round the basket", f=90.0)
        yield ("snap", "6_captured_on_tray")
        yield self._grab(k, P3.BASKET_TRAY_Y0, dst)
        yield ("slow", self.mv(f"servo 1 out: the rear fingers push the basket onto the {what}", arm=P3.TIP_PUSHED))

        def settled():
            if dst[0] == "shelf":
                self.slot[k] = dst[1]
                self.reserved.discard(dst[1])
                self.basket[k] = "shelf"
            else:
                self.dock_of[k] = dst[1]
                self.basket[k] = "dock"
        yield ("call", settled)
        yield self.mv("back off 1 mm, fingers swing up", arm=P3.TIP_SHELF, f=0.0)
        yield self.mv("arms retract", arm=P3.TIP0)

    def park(self):
        yield from self.to_flat()
        yield from self.go_home()

    def next_slot(self):
        for s in self.slots + self.ALL_SLOTS:
            if self.id_at(*s) is not None:
                return s

    # ------------------------------------------------------------------ weight balance (static estimate, shown live)
    M_BODY, M_TRAY, M_BASKET = 1.3, 1.0, 0.3        # kg: column + hubs + motors, tray + arms, empty basket (ESTIMATES)

    def masses(self):
        """(name, kg, (x, y, z) mm in the robot frame: origin between the hubs, x along the rails, y into the rack)"""
        out = [("robot body", self.M_BODY, (0.0, -85.0, 0.0)),
               ("tray + arms", self.M_TRAY, (P3.BXR, -170.0, P3.T_V3 + 30))]
        if self.bat_on:
            out.append(("battery", P3.BAT_MASS, ((P3.BAT_X[0] + P3.BAT_X[1]) / 2, (P3.BAT_Y[0] + P3.BAT_Y[1]) / 2, 0.0)))
        Ri = np.linalg.inv(self.robot_M())
        for (b, l), st in self.basket.items():
            if st not in ("tray", "held"):
                continue
            p = (Ri @ np.r_[self.d.xpos[self.m.body(f"basket_{b}_{l}").id] / MM, 1.0])[:3]
            y0 = p[1]
            frac = 1.0 if st == "tray" else max(0.0, min(1.0, (P3.TRAY_Y[1] - y0) / P3.BASKET_D))   # share already on the tray
            if frac > 0:
                yc = max(y0, P3.TRAY_Y[1] - P3.BASKET_D * frac) + P3.BASKET_D * frac / 2 if st != "tray" else y0 + P3.BASKET_D / 2
                out.append(("basket", (self.M_BASKET + MASS[(b, l)]) * frac, (p[0], min(yc, P3.TRAY_Y[1]), p[2] + 55)))
        return out

    def balance(self):
        ms = self.masses()
        W = sum(m for _, m, _ in ms) * G
        cog = np.sum([np.array(p) * m for _, m, p in ms], axis=0) / sum(m for _, m, _ in ms)
        mx = sum(m * G * p[0] for _, m, p in ms) / 1000                    # twist about the column (N*m), + = tray side down
        my = sum(m * G * (-50.0 - p[1]) for _, m, p in ms) / 1000          # forward tipping away from the rails (N*m)
        D = P3.D_HUB / 1000
        return dict(W=W, cog=cog, mx=mx, my=my, f_twist=mx / D, f_fwd=my / D)

    def draw_balance(self, scn):
        """centre of gravity, weight, and the rail forces on the two hubs, as arrows in the 3-D view.
        Drawn on a plane just in front of the robot (projection) so nothing hides them.  White dots = hubs."""
        if not self.show_balance:
            return
        bl = self.balance()
        S = 6.0                                                               # mm of arrow per newton
        Rw = self.robot_M()
        W = lambda p: (Rw @ np.r_[np.asarray(p, float), 1.0])[:3]            # robot frame -> world (mm)

        def add(kind, rgba, p0=None, p1=None, size=None, pos=None):
            if scn.ngeom >= scn.maxgeom:
                return
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, kind, np.array(size if size is not None else [0, 0, 0], float),
                                np.array(pos if pos is not None else [0, 0, 0], float) * MM, np.eye(3).flatten(), np.array(rgba, float))
            if p0 is not None:
                mujoco.mjv_connector(g, kind, 0.006 if kind == mujoco.mjtGeom.mjGEOM_ARROW else 0.003,
                                     np.array(p0, float) * MM, np.array(p1, float) * MM)
            g.category = mujoco.mjtCatBit.mjCAT_DECOR
            scn.ngeom += 1
        YF = P3.TRAY_Y[0] - 70.0                                           # draw on a plane just in front of the robot
        cog = np.array([bl["cog"][0], YF, bl["cog"][2]])
        add(mujoco.mjtGeom.mjGEOM_SPHERE, (1, 0.85, 0, 1), size=[0.016, 0, 0], pos=W(cog))
        add(mujoco.mjtGeom.mjGEOM_CAPSULE, (1, 0.85, 0, 1), p0=W((0, cog[1], cog[2])), p1=W(cog))      # lever arm to the column
        add(mujoco.mjtGeom.mjGEOM_ARROW, (1, 0.85, 0, 1), p0=W(cog), p1=W((cog[0], cog[1], cog[2] - bl["W"] * 1.5)))  # weight
        for sgn, dz in ((1, P3.D_HUB / 2), (-1, -P3.D_HUB / 2)):
            hp = np.array([0.0, YF, dz])
            add(mujoco.mjtGeom.mjGEOM_SPHERE, (1, 1, 1, 1), size=[0.008, 0, 0], pos=W(hp))
            ft, ff = -sgn * bl["f_twist"], sgn * bl["f_fwd"]                 # rail forces on the hub
            if abs(ft) > 0.3:
                add(mujoco.mjtGeom.mjGEOM_ARROW, (0.2, 0.45, 1, 1), p0=W(hp), p1=W(hp + [ft * S, 0, 0]))
            if abs(ff) > 0.3:
                add(mujoco.mjtGeom.mjGEOM_ARROW, (1, 0.2, 0.2, 1), p0=W(hp), p1=W(hp + [0, ff * S, 0]))

    def balance_text(self):
        bl = self.balance()
        cog = bl["cog"]
        return ("BALANCE (static estimate)",
                f"total weight {bl['W']:.0f} N, payload {self.payload:.1f} kg\n"
                f"centre of gravity {cog[0]:+.0f} mm beside the column,\n"
                f"  {-50 - cog[1]:.0f} mm in front of the rails\n"
                f"twist {bl['mx']:+.2f} N*m -> {abs(bl['f_twist']):.0f} N sideways on each hub (blue)\n"
                f"forward tip {bl['my']:.2f} N*m -> {abs(bl['f_fwd']):.0f} N in/out on each hub (red)\n"
                f"battery counterweight: {'ON' if self.bat_on else 'OFF (ignored)'}")

    # ------------------------------------------------------------------ scheduler
    def run_task(self, n):
        if n not in self.tasks:
            return
        name, fn = self.tasks[n]
        if fn is None:
            self.reset(); self.say("reset"); return
        self.queue.append((n, name, fn))
        self.say(f"queued task {n}: {name}")

    @property
    def busy(self):
        return self.gen is not None or self.move is not None or bool(self.queue)

    def tick(self, dt):
        self.t += dt
        if self.move is None and self.gen is None and self.queue:
            self.from_sliders()
            n, name, fn = self.queue.popleft()
            self.task_name = f"task {n}: {name}"
            self.say(f"start {self.task_name}")
            self.gen = fn()
        while self.move is None and self.gen is not None:
            try:
                it = next(self.gen)
            except StopIteration:
                self.gen = None; self.say(f"done  {self.task_name}"); self.task_name, self.step_name = "idle", ""
                break
            slow = it[0] == "slow"
            if slow: it = it[1]
            if it[0] == "call":
                r = it[1]()
                if r is not None and hasattr(r, "__next__"):       # a call that returns more steps (e.g. auto-balance)
                    rest = self.gen
                    self.gen = self._chain(r, rest)
                continue
            if it[0] == "snap": self.snaps.append(it[1]); continue
            if it[0] == "pause":
                self.step_name = it[1]
                self.move = dict(t=0.0, dur=it[2], a=dict(self.cmd), b=dict(self.cmd))
                break
            if it[0] == "dyn": it = ("move", it[1], it[2]())
            _, self.step_name, tgt = it
            dur = 0.3
            for k, v in tgt.items():
                sp = SPEED[speed_key(k)] * self.speed * self.slowmo * (0.6 if slow and k == "arm" else 1)
                dur = max(dur, abs(v - self.cmd[k]) / sp)
            self.move = dict(t=0.0, dur=dur, a=dict(self.cmd), b={**self.cmd, **tgt})
        if self.move is not None:
            mvd = self.move
            mvd["t"] += dt
            u = min(1.0, mvd["t"] / mvd["dur"])
            s = u ** 3 * (10 - 15 * u + 6 * u * u)
            for k in self.cmd:
                self.cmd[k] = mvd["a"][k] + (mvd["b"][k] - mvd["a"][k]) * s
            if u >= 1.0:
                self.move = None
        elif not self.busy:
            self.from_sliders()
        self.apply()

    @staticmethod
    def _chain(a, b):
        yield from a
        yield from b

    def overlay(self):
        c = self.cmd
        state = (f"{self.task_name}\n{self.step_name}\n\n"
                 f"row {self.row + 1}  x {c['x']:.0f}  z {c['z']:.0f}  cart y {c['cart']:.0f} mm   hubs {c['h1']:.0f}/{c['h2']:.0f} deg\n"
                 f"payload {self.payload:.1f} kg (load cell)")
        # the task list lives only on the web page; the viewer shows just what the robot is doing
        t, b = self.balance_text()
        return [(mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, "NOW", state),
                (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT, t, b)]


def set_cam(cam, az=125, el=-35, dist=4.6, look=(0.75, 0.55, 0.3)):
    cam.azimuth, cam.elevation, cam.distance = az, el, dist
    cam.lookat[:] = look


def headless(args):
    sim = Sim(args.speed)
    r = mujoco.Renderer(sim.m, 720, 1280)
    cam = mujoco.MjvCamera()
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "snaps_v3")
    os.makedirs(out, exist_ok=True)
    import PIL.Image as I

    def snap(name):
        views = [("", 125, -35, 4.6), ("_close", 125, -12, 1.2)]       # whole rack, then close on the robot (front-right)
        if "balance" in name:
            views.append(("_front", 90, -3, 1.1))                       # straight on: the balance arrows read best
        for tag, az, el, dist in views:
            look = (0.75, 0.55, 0.3) if not tag else tuple(sim.d.xpos[sim.m.body("robot").id] + sim.robot_M()[:3, 0] * 0.12)
            set_cam(cam, az=az, el=el, dist=dist, look=look); r.update_scene(sim.d, cam); sim.draw_balance(r.scene)
            p = os.path.join(out, name + tag + ".png"); I.fromarray(r.render()).save(p)
        print("  snapshot", os.path.join(out, name + "[_close].png"))

    snap("0_home")
    for n in args.task:
        sim.run_task(n)
        done = len(sim.snaps)
        while sim.busy:
            sim.tick(0.02)
            while done < len(sim.snaps):
                snap(sim.snaps[done]); done += 1
    print(f"sim time {sim.t:.0f} s")


def viewer(args):
    import mujoco.viewer
    sim = Sim(args.speed)
    keys = {"6": 3, "7": 5, "8": 8, "9": 10}
    from remote import Remote
    rc = Remote({n: name for n, (name, _) in sim.tasks.items()}, port=args.port, browser=not args.no_browser)

    def key(k):
        c = chr(k) if 0 < k < 128 else ""
        if c in keys:
            sim.run_task(keys[c])

    print(f"task buttons: {rc.url}", flush=True)
    with mujoco.viewer.launch_passive(sim.m, sim.d, key_callback=key) as v:
        set_cam(v.cam)
        last = time.time()
        while v.is_running():
            now = time.time()
            for n in rc.poll():
                sim.run_task(n)
            with v.lock():
                sim.tick(min(now - last, 0.05))
                v.user_scn.ngeom = 0
                sim.draw_balance(v.user_scn)
            rc.status = dict(task=sim.task_name, step=sim.step_name, queued=len(sim.queue),
                             queue=[f"{n}: {name}" for n, name, _ in sim.queue], log=list(sim.log))
            v.set_texts(sim.overlay())
            last = now
            v.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--task", type=lambda s: [int(i) for i in s.split(",")], default=[1], help="headless: task numbers, e.g. 1,4,5")
    ap.add_argument("--out", default="")
    ap.add_argument("--port", type=int, default=8765, help="viewer: port of the task-button page")
    ap.add_argument("--no-browser", action="store_true", help="viewer: do not open the task page automatically")
    a = ap.parse_args()
    (headless if a.headless else viewer)(a)
