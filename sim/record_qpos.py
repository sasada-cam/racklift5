"""Record a MuJoCo sim's motion (qpos over time) so a web page can play it back - the physics engine stays MuJoCo.

    .pixi/envs/sim/bin/python sim/record_qpos.py --kind v3 --root <source tree> --tasks 1,4,7,13 --out sim/build/traj/x.json
    .pixi/envs/sim/bin/python sim/record_qpos.py --kind v2 --root <source tree> --out sim/build/traj/v2.json

--root is the source tree whose sim/run_v3.py (v3) or sim/run.py (v2) is run: the repo itself, or a git worktree of an
older commit (e.g. the original v3 at e40004d), so every version is recorded with its own code.

Output: {kind, nq, joints: [{name, type, adr}], clips: [{name, dt, start: [full qpos], frames: [[[i, v], ...] per frame]}]}
Frames are sparse (only the qpos entries that changed since the previous frame), sampled every dt of sim time.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def load_runner(root, kind):
    sys.path[:] = [p for p in sys.path if os.path.realpath(p or ".") not in (os.path.realpath(HERE),)]
    sys.path.insert(0, os.path.join(root, "generator"))
    sys.path.insert(0, os.path.join(root, "sim"))
    os.chdir(root)
    import importlib
    return importlib.import_module("run_v3" if kind == "v3" else "run")


def joint_table(m, mujoco):
    names = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}
    return [{"name": m.joint(i).name, "type": names[int(m.jnt_type[i])], "adr": int(m.jnt_qposadr[i])} for i in range(m.njnt)]


class Recorder:
    def __init__(self, d, dt):
        self.d, self.dt, self.clips = d, dt, []

    def start(self, name):
        self.cur = {"name": name, "dt": self.dt, "start": [round(float(v), 5) for v in self.d.qpos], "frames": []}
        self.prev = list(self.cur["start"])
        self.clips.append(self.cur)

    def sample(self):
        now = [round(float(v), 5) for v in self.d.qpos]
        self.cur["frames"].append([[i, v] for i, (v, p) in enumerate(zip(now, self.prev)) if v != p])
        self.prev = now


def record_v3(R, tasks, dt):
    import mujoco
    sim = R.Sim(1.0)
    rec = Recorder(sim.d, dt)
    for n in tasks:
        if n not in sim.tasks:
            print(f"  (task {n} not in this version - skipped)")
            continue
        name = sim.tasks[n][0]
        rec.start(f"{n}: {name}")
        sim.run_task(n)
        step = 0.02
        t_next = sim.t + dt                         # (sim.t keeps running across clips)
        while sim.busy:
            sim.tick(step)
            while sim.t >= t_next - 1e-9:
                rec.sample(); t_next += dt
        rec.sample()
        print(f"  task {n}: {len(rec.cur['frames'])} frames ({len(rec.cur['frames']) * dt:.0f} s)  {name}")
    return sim.m, rec.clips, mujoco


def record_v2(R, dt):
    import mujoco
    sim = R.Sim()
    rec = Recorder(sim.d, dt)
    clips = [("fetch bay 1 shelf 3", (0, 2))]          # (a second fetch does nothing: the tray already holds the first basket)
    for name, (b, l) in clips:
        rec.start(name)
        sim.enqueue(lambda b=b, l=l: sim.fetch(b, l))
        t_next = sim.d.time + dt
        while sim.busy and sim.d.time < 600.0:
            sim.step()
            while sim.d.time >= t_next - 1e-9:
                rec.sample(); t_next += dt
        rec.sample()
        print(f"  {name}: {len(rec.cur['frames'])} frames ({len(rec.cur['frames']) * dt:.0f} s)")
    return sim.m, rec.clips, mujoco


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kind", choices=("v2", "v3"), required=True)
    ap.add_argument("--root", required=True, help="source tree to run (repo or a git worktree of an older commit)")
    ap.add_argument("--tasks", default="1,4,7,13,15,17", help="v3: task numbers, run one after the other")
    ap.add_argument("--dt", type=float, default=0.1, help="sample period (s of sim time)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    R = load_runner(os.path.abspath(a.root), a.kind)
    if a.kind == "v3":
        m, clips, mujoco = record_v3(R, [int(t) for t in a.tasks.split(",")], a.dt)
    else:
        m, clips, mujoco = record_v2(R, a.dt)
    data = {"kind": a.kind, "nq": int(m.nq), "joints": joint_table(m, mujoco), "clips": clips}
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        json.dump(data, f, separators=(",", ":"))
    print(f"-> {out}  ({os.path.getsize(out) / 1e6:.1f} MB, {len(clips)} clips, nq {m.nq})")


if __name__ == "__main__":
    main()
