"""Interactive MuJoCo simulation of the rigged 4632-370 (12V 100 RPM) worm gear motor,
with its 3D-printed PETG cover and 14T climber pinion head.

Usage:
  .pixi/envs/sim/bin/mjpython sim/run_4632_370.py          # interactive window (auto-cycle)
  .pixi/envs/sim/bin/mjpython sim/run_4632_370.py --manual # control via viewer slider
  .pixi/envs/sim/bin/python sim/run_4632_370.py --headless # headless smoke test
"""

import argparse
import math
import os
import time
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
XML_PATH = os.path.join(HERE, "4632_370.xml")
RPM_100 = 100.0 * 2.0 * math.pi / 60.0  # 10.472 rad/s


def target_velocity(sim_time: float, rpm: float) -> float:
    """Cycle: Forward (3s) -> Hold (1s) -> Reverse (3s) -> Hold (1s)."""
    w = rpm * 2.0 * math.pi / 60.0
    phase = sim_time % 8.0
    if phase < 3.0:
        return w
    if phase < 4.0:
        return 0.0
    if phase < 7.0:
        return -w
    return 0.0


def main():
    parser = argparse.ArgumentParser(description="Simulate the rigged 4632-370 worm gear motor")
    parser.add_argument("--rpm", type=float, default=100.0, help="Target speed in RPM (default: 100)")
    parser.add_argument("--manual", action="store_true", help="Use the MuJoCo viewer control slider instead of auto-cycling")
    parser.add_argument("--headless", action="store_true", help="Run a 4-second headless check without opening a window")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    if args.headless:
        for _ in range(2000):  # 4.0 s at dt = 0.002
            data.ctrl[0] = target_velocity(data.time, args.rpm)
            mujoco.mj_step(model, data)
        rpm_now = data.qvel[0] * 60.0 / (2.0 * math.pi)
        print(f"Headless check ok: t={data.time:.2f}s, angle={math.degrees(data.qpos[0]):.1f} deg, speed={rpm_now:.1f} RPM")
        return

    from mujoco import viewer as mj_viewer

    with mj_viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.distance = 0.16
        viewer.cam.lookat[:] = [-0.01, 0.0, 0.02]

        while viewer.is_running():
            t0 = time.perf_counter()
            if not args.manual:
                data.ctrl[0] = target_velocity(data.time, args.rpm)
            mujoco.mj_step(model, data)
            viewer.sync()
            dt = model.opt.timestep - (time.perf_counter() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
