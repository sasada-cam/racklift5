"""RackLift5 v2 in MuJoCo - built-in viewer.

    pixi run -e sim sim                       # open the viewer (macOS: uses mjpython)
    pixi run -e sim sim-headless              # no window: run a fetch and print the loads
    options: --payload KG  --counterweight KG  --speed X  --fetch BAY,LEVEL (0-based)  --nobasket

In the viewer:
  * right panel -> "Control": drag the sliders (x, z, turn_body, turntable_*, arm_*). hand_level is automatic.
  * keys   6 = fetch next basket to the home spot      7 = operator takes the basket off the tray
           8 = demo (fetch + unload, twice)            9 = reset
"""
import argparse, math, os, sys, time
from collections import deque
import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import model as MD
P, A = MD.P, MD.A

MMs = 1e-3
JZ, T = P.JZ * MMs, P.TRAY_TOP_REL * MMs
PIN_STOW = -178.0
SPEED = dict(x=0.25, z=0.12, th=0.7, pin=0.10)        # m/s, rad/s (real climb is ~0.024 m/s - see README)
HALF_PI = math.pi / 2


def arm_q(pin_y_mm):
    q1, E, q2 = A.ik(pin_y_mm)
    rel = (q2 - q1 + math.pi) % (2 * math.pi) - math.pi
    return q1, rel


def pin_from_q(q1, rel):
    return (P.SHOULDER[1] + P.L1 * math.sin(q1) + P.L2 * math.sin(q1 + rel))


class Sim:
    def __init__(self, payload=0.5, counterweight=0.0, speed=1.0):
        path, self.info = MD.build(payload, counterweight)
        self.m = mujoco.MjModel.from_xml_path(path)
        self.d = mujoco.MjData(self.m)
        self.speed = speed
        j = lambda n: self.m.joint(n).qposadr[0]
        self.q = {n: j(n) for n in ("x", "z", "theta", "rotor0", "rotor1", "hand", "q1", "q2")}
        self.a = {n: self.m.actuator(n).id for n in ("x", "z", "turn_body", "turntable_1", "turntable_2", "hand_level", "arm_shoulder", "arm_elbow")}
        self.slots = [(0, 2), (1, 4), (0, 0), (1, 3), (0, 4), (1, 1)]
        self.next_slot = 0
        self.log = []
        self.reset()

    # ----------------------------------------------------------------- state
    def reset(self):
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        xh = self.info["x_home"] * MMs
        self.cmd = dict(x=xh, z=JZ, th=-HALF_PI, r0=-HALF_PI, r1=-HALF_PI, pin=PIN_STOW)
        q1, rel = arm_q(PIN_STOW)
        for n, v in dict(x=xh, z=JZ, theta=-HALF_PI, rotor0=-HALF_PI, rotor1=-HALF_PI, hand=HALF_PI, q1=q1, q2=rel).items():
            d.qpos[self.q[n]] = v
        self.apply_cmd()
        mujoco.mj_forward(m, d)
        self.queue = deque(); self.gen = None; self.move = None
        self.tray_basket = None
        self.peak = dict(hand=0.0, turn=0.0)

    def apply_cmd(self):
        c, u, a = self.cmd, self.d.ctrl, self.a
        u[a["x"]], u[a["z"]], u[a["turn_body"]] = c["x"], c["z"], c["th"]
        u[a["turntable_1"]], u[a["turntable_2"]] = c["r0"], c["r1"]
        q1, rel = arm_q(c["pin"])
        u[a["arm_shoulder"]], u[a["arm_elbow"]] = q1, rel

    def sync_cmd_from_ctrl(self):
        u, a, c = self.d.ctrl, self.a, self.cmd
        c.update(x=u[a["x"]], z=u[a["z"]], th=u[a["turn_body"]], r0=u[a["turntable_1"]], r1=u[a["turntable_2"]],
                 pin=pin_from_q(u[a["arm_shoulder"]], u[a["arm_elbow"]]))

    def basket_id(self, b, l):
        return self.m.body(f"basket_{b}_{l}").id

    def basket_pos(self, b, l):
        return self.d.xpos[self.basket_id(b, l)].copy()

    # ----------------------------------------------------------------- task generators (yield Move or ('call', fn))
    def mv(self, **tgt):
        return ("move", tgt)

    def normalize(self):
        c = self.cmd
        j = 0 if abs(c["x"] - P.RAIL_X[0] * MMs) < abs(c["x"] - P.RAIL_X[1] * MMs) else 1
        if c["th"] > -1.0:                                           # robot is on a vertical rail: come down and turn flat
            yield self.mv(pin=PIN_STOW)
            yield self.mv(x=P.RAIL_X[j] * MMs)
            yield self.mv(z=JZ)
            yield self.mv(th=-HALF_PI, **{f"r{j}": -HALF_PI})
        yield self.mv(pin=PIN_STOW)

    def fetch(self, b, l):
        if self.tray_basket is not None:
            self.say("tray is full - press 7 first"); return
        if (b, l) not in self.slots_present():
            self.say(f"bay {b+1} level {l+1} is empty"); return
        self.say(f"fetch: bay {b+1} level {l+1}")
        yield from self.normalize()
        zl = P.LEVELS[l] * MMs - T
        r = f"r{b}"
        yield self.mv(x=P.RAIL_X[b] * MMs)
        yield self.mv(th=0.0, **{r: 0.0})                              # turntable swings the robot upright
        yield self.mv(z=zl + P.APPROACH * MMs)                         # climb, pin passes 5 mm over the rim
        yield self.mv(pin=P.BASKET_Y0 + 15)                            # reach over the basket's front wall
        yield self.mv(z=zl)                                            # lower: hook sits behind the wall
        yield ("slow", self.mv(pin=PIN_STOW))                          # pull the basket onto the tray
        yield ("call", lambda: setattr(self, "tray_basket", (b, l)))
        yield self.mv(z=JZ)
        yield self.mv(th=-HALF_PI, **{r: -HALF_PI})
        yield self.mv(x=self.info["x_home"] * MMs)
        self.say("delivered - press 7 to take the basket")

    def unload(self):
        if self.tray_basket is None:
            self.say("nothing on the tray"); return
        b, l = self.tray_basket
        adr = self.m.joint(f"bj_{b}_{l}").qposadr[0]
        n = len([1 for k in self.slots if k not in self.slots_present()])
        self.d.qpos[adr:adr + 7] = [self.info["x_home"] * MMs + 0.75 + 0.3 * n, -0.55, 0.003, 1, 0, 0, 0]
        self.d.qvel[self.m.joint(f"bj_{b}_{l}").dofadr[0]:][:6] = 0
        self.removed.add((b, l)); self.tray_basket = None
        self.say(f"basket bay {b+1} L{l+1} taken")
        yield ("call", lambda: None)

    def demo(self):
        for _ in range(2):
            slot = self.pick_slot()
            if slot is None: break
            yield from self.fetch(*slot)
            yield self.mv()
            yield from self.unload()

    removed = set()

    def slots_present(self):
        return [(b, l) for b in range(P.N_BAYS) for l in range(len(P.LEVELS))
                if (b, l) not in self.removed and (b, l) != self.tray_basket]

    def pick_slot(self):
        pres = self.slots_present()
        for s in self.slots:
            if s in pres: return s
        return pres[0] if pres else None

    def say(self, s):
        print(f"[{self.d.time:6.1f}s] {s}", flush=True)

    # ----------------------------------------------------------------- scheduler
    def enqueue(self, gen_fn):
        self.queue.append(gen_fn)

    def start_move(self, tgt, slow=False):
        self.sync_move_start = dict(self.cmd)
        tgt = {k: v for k, v in tgt.items()}
        dur = 0.3
        sp = dict(SPEED)
        if slow: sp["pin"] *= 0.6
        for k, v in tgt.items():
            key = {"x": "x", "z": "z", "th": "th", "r0": "th", "r1": "th", "pin": "pin"}[k]
            dist = abs(v - self.cmd[k]) * (MMs if k == "pin" else 1)
            dur = max(dur, dist / (sp[key] * self.speed))
        self.move = dict(t0=self.d.time, dur=dur, a=dict(self.cmd), b={**self.cmd, **tgt})

    def advance(self):
        d = self.d
        if self.move is None:
            if self.gen is None and self.queue:
                self.sync_cmd_from_ctrl()
                self.gen = self.queue.popleft()()
            if self.gen is not None:
                try:
                    while True:
                        item = next(self.gen)
                        slow = False
                        if item[0] == "slow": slow, item = True, item[1]
                        if item[0] == "call": item[1](); continue
                        self.start_move(item[1], slow); break
                except StopIteration:
                    self.gen = None
        if self.move is not None:
            mv = self.move
            u = min(1.0, (d.time - mv["t0"]) / mv["dur"])
            s = u**3 * (10 - 15 * u + 6 * u * u)
            for k in self.cmd:
                self.cmd[k] = mv["a"][k] + (mv["b"][k] - mv["a"][k]) * s
            self.apply_cmd()
            if u >= 1.0: self.move = None
        # the hand keeps the tray level (IMU-style): hand angle = -body angle
        lo, hi = self.m.actuator_ctrlrange[self.a["hand_level"]]
        d.ctrl[self.a["hand_level"]] = float(np.clip(-d.qpos[self.q["theta"]], lo, hi))

    def step(self):
        self.advance()
        mujoco.mj_step(self.m, self.d)
        f = self.d.actuator_force
        self.peak["hand"] = max(self.peak["hand"], abs(f[self.a["hand_level"]]))
        self.peak["turn"] = max(self.peak["turn"], abs(f[self.a["turn_body"]]))

    @property
    def busy(self):
        return self.gen is not None or self.move is not None or bool(self.queue)

    def run_until_idle(self, limit=600.0):
        while self.busy and self.d.time < limit:
            self.step()


def tray_report(sim):
    b, l = sim.tray_basket if sim.tray_basket else (None, None)
    if b is None: return "no basket on tray"
    p = sim.basket_pos(b, l)
    hand = sim.d.xpos[sim.m.body("hand").id]
    rel = (p - hand) * 1000
    return f"basket on tray at dx={rel[0]:.0f} dy={rel[1]:.0f} dz={rel[2]:.0f} mm from the hand axle"


def headless(args):
    sim = Sim(args.payload, args.counterweight, args.speed)
    b, l = args.fetch
    sim.enqueue(lambda: sim.fetch(b, l))
    t0 = time.time()
    sim.run_until_idle()
    sim.say(tray_report(sim))
    # hold torque with the basket on the tray, robot parked horizontally
    for _ in range(1500): sim.step()
    f = sim.d.actuator_force
    print(f"\nload case: payload {args.payload} kg in the basket, counterweight {args.counterweight} kg")
    print(f"  hand axle torque (holding the tray level): {abs(f[sim.a['hand_level']]):.2f} N*m   peak {sim.peak['hand']:.2f}")
    print(f"  body/rail reaction moment                : {abs(f[sim.a['turn_body']]):.2f} N*m   peak {sim.peak['turn']:.2f}")
    print(f"  sim {sim.d.time:.0f} s in {time.time()-t0:.0f} s wall")
    if args.png:
        r = mujoco.Renderer(sim.m, 720, 1280)
        cam = mujoco.MjvCamera(); set_cam(cam)
        r.update_scene(sim.d, cam); import PIL.Image as I; I.fromarray(r.render()).save(args.png); print("saved", args.png)


def set_cam(cam, az=90, el=-12, dist=2.6):
    cam.azimuth, cam.elevation, cam.distance = az, el, dist
    cam.lookat[:] = [0.1, 0.0, 0.5]


def viewer(args):
    import mujoco.viewer
    sim = Sim(args.payload, args.counterweight, args.speed)

    def key(k):
        c = chr(k) if 0 < k < 128 else ""
        if c == "6":
            s = sim.pick_slot()
            if s: sim.enqueue(lambda s=s: sim.fetch(*s))
        elif c == "7": sim.enqueue(sim.unload)
        elif c == "8": sim.enqueue(sim.demo)
        elif c == "9": sim.removed = set(); sim.reset()

    print(__doc__)
    with mujoco.viewer.launch_passive(sim.m, sim.d, key_callback=key) as v:
        set_cam(v.cam)
        t_wall = time.time(); t_sim = sim.d.time
        while v.is_running():
            with v.lock():
                target = t_sim + (time.time() - t_wall)
                if sim.d.time > target + 1: t_wall, t_sim = time.time(), sim.d.time   # after reset
                n = 0
                while sim.d.time < target and n < 40:
                    sim.step(); n += 1
            v.sync()
            time.sleep(0.004)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--payload", type=float, default=0.5, help="kg of parts in the basket")
    ap.add_argument("--counterweight", type=float, default=0.0, help="kg on the opposite side of the hand axle")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--fetch", type=lambda s: tuple(int(i) for i in s.split(",")), default=(0, 2))
    ap.add_argument("--png", default="")
    a = ap.parse_args()
    (headless if a.headless else viewer)(a)
