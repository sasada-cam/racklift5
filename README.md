# RackLift 5 (v2)

A climbing rack-and-pinion robot for a slotted-angle shelf rack. One robot climbs vertical V-slot rails
with in-slot printed gear racks, turns 90° on turntable junctions to drive along the bottom rail, keeps its
tray level, and picks baskets off 5 shelf levels with a 2-link SCARA hook arm.

![reach pose](docs/renders/v2_01_reach.png)

## Repository layout

| Path | What it is |
|---|---|
| `generator/` | **Parametric source of the design** (Python + numpy). `params.py` holds every dimension; values marked `# MEASURE` must be checked with a caliper. Original mesh pipeline: `export_v2.py` (STL), `clearance_v2.py` (67-pose collision check), `r2.py` (renders). |
| `cad/` | **Exact CAD export.** `cad/brep/csg.py` is a drop-in build123d / OpenCascade backend for `generator/csg.py`, so the unchanged generator code produces true B-rep solids (real cylinders, slots, involute gear splines). |
| `models/print/` | Print-ready STEP + STL, already in print orientation (mm). |
| `models/design/` | Each part as STEP in its design frame – use these for FEA. |
| `models/assembly/` | Whole machine in 3 poses (`reach`, `travel`, `turning`) as STEP + FreeCAD `.FCStd`: 97 named, coloured bodies in 5 groups. |
| `models/parts_list.csv` | Quantities, sizes and print notes per part. |
| `models/parts_report.csv` | Volume, solid PETG mass, size and validity of each exact solid. |
| `docs/` | Renders and the original v2 release notes. |
| `archive/v1/` | RackLift 5 v1 (two-rail lift), kept for reference only. |
| `archive/v2_original_stl/` | The original mesh STLs of v2 – reference for the volume check. |

## Setup (Windows, macOS, Linux)

The project uses **[pixi](https://pixi.sh)**: one lock file (`pixi.lock`) pins Python, the CAD kernel
(build123d / OCP), the mesher (gmsh) and the FEM solver (CalculiX `ccx`) for `win-64`, `osx-arm64`
(Apple Silicon), `osx-64` (Intel Mac) and `linux-64`. No system Python is needed.

### 1. Install the tools (once per machine)

| | Windows (PowerShell) | macOS / Linux (Terminal) |
|---|---|---|
| Git + Git LFS | `winget install Git.Git GitHub.GitLFS` | `brew install git git-lfs` |
| pixi | `winget install prefix-dev.pixi` (or `scoop install pixi`) | `curl -fsSL https://pixi.sh/install.sh \| sh` |

Then once: `git lfs install`. Open a **new** terminal afterwards so the PATH is refreshed.

### 2. Clone and install the environment
```
git clone https://github.com/sasada-cam/racklift5.git
cd racklift5
pixi install                 # default env: CAD export + FEM toolchain (~0.5 GB)
pixi run check-tools         # smoke test: prints build123d / gmsh versions and runs ccx
```
Optional environments (only if you need them – they are large):
```
pixi install -e viz          # + pyvista for 3-D result plots (~1 GB, VTK)
pixi install -e freecad      # + headless FreeCAD 1.0 (~1.5 GB)
```

### 3. Everyday commands
```
pixi task list               # everything available
pixi run export              # all parts + 3 assembly poses -> models/   (~1 min)
pixi run export-parts        # only the printable parts
pixi run test                # validity + volume check vs archive/v2_original_stl
pixi run -e freecad verify   # open all STEP files in FreeCAD, write assembly .FCStd
pixi run gen-stl             # original mesh pipeline -> generator/out/ (git-ignored)
pixi run gen-clearance       # original 67-pose collision check
```
Workflow: edit `generator/params.py` → `pixi run export` → `pixi run test` → commit.

Without pixi (CAD export only, no CalculiX solver): `pip install .` (or `pip install ".[fem]"` to add gmsh/meshio) with Python 3.11–3.14. Dependencies are declared in `pyproject.toml`.

### v3: compact Squid layout (work in progress)
```
        ||  vertical rail (one per bay)              top view:
   [H1]=++=============  upper horizontal rail      ======+=================  rails
    |   ||   [tray]                                   [bat]|col| +------------+
 [bat]=col=                                                      | || basket || |  || = telescopic arms
   [H2]=++=============  lower horizontal rail                   +------------+
```
* **Rails:**
  * Travel: one hub rides each of the **two horizontal rails** (z 70 and 330, 260 mm apart).
  * Climbing: both hubs climb the **single vertical rail**.
  * Every rail crossing has a small turntable piece on a 50 mm bearing. A hub **swivels 90 deg** together with its piece,
    one hub at a time, while the other stays locked.
  * The body never rotates, so the tray is always level.
* **Picker:** two 2-stage **telescopic arms** run in the 23 mm gaps beside the basket. **Two servos drive everything**, and
  every part of the mechanism is modelled and moves in the sim:
  * **Arms (servo 1, DS3218 270 deg):** three 13 x 8 mm bars stacked on PTFE pads (outer fixed, middle, inner).
    Servo 1 -> GT2 step-up belt (25T -> 8T) -> cross shaft (locks both arms together) -> a belt loop beside each arm,
    with the middle bar clamped to its top run.
  * **2:1 stage:** two pulleys on the middle bar. The *extend* belt is anchored to the rear of the outer and inner bars,
    the *retract* belt to their fronts, so the inner bar travels twice as far as the middle bar (300 mm reach from 150 mm).
    A rack-and-pinion idler cannot do this with 180 mm bars: no single place on the middle bar stays meshed with both
    racks over the whole stroke.
  * **Fingers (servo 2, MG90S):** each arm has a telescoping square shaft (fixed rod / middle tube / inner tube, like a
    PTO shaft) carrying a front and a rear finger. Servo 2 turns the left rod through a spur-gear pair (opposite way)
    and the right rod through a belt across the tray front (same way), so both sides swing inward.
  * **Pick:** both fingers drop together and capture the basket with 2 mm play: front pair behind the far wall, rear
    pair in front of the near wall. Retracting pulls the basket onto the tray; extending pushes it back.
  * The tray stays in front of the rails; the drive sits on a front strip (robot is ~45 mm longer to the front).
* **Battery and electronics:** a fixed box beside the column, opposite the tray and close to the rails, acts as a passive
  counterweight. A load cell under the tray weighs the payload (overload check).
* **Floor of racks:** 3 rows of 4 racks (12 bays, 60 baskets), all rows facing the same way, rails on each row's
  front. A **transfer cart** at the rows' right ends carries the robot between rows (the red joints + side bar of the
  sketch):
  ```
  top view          row 3  [r][r][r][r]==|C|   C = cart stop (two short rail pieces on a cart)
                           aisle         |#|   # = cart track on the floor + fixed transfer gear rack on a beam
                    row 2  [r][r][r][r]==|C|
                           aisle         |#|
                    row 1  [r][r][r][r]==|C|  <- home: row 1, operator side
  ```
  * The robot drives off the end of its row onto the cart's rail pieces. An SG90 drops the robot's own 16T pinion into
    the fixed rack along the track, and the pinion pushes robot + cart across the aisles (the self-locking hub motors
    hold the robot on the cart). At the new row the pinion lifts and the robot drives on. The cart has **no motor**.
  * The robot never turns, so all rows face the same way. The cart is the only way between rows, so it is always at
    the robot's row.
  * Why not corner turrets (as tried for the 4-rack tower): the robot is not mirror-symmetric (tray on one side of the
    column), so at a T-junction its body sweeps over the side rail behind it, and left-hand corners only work one way.
    Turning corners works only at plain L corners (a ring of racks round a block); the cart works for any number of rows.
  * Rows are 650 mm apart (230 rack + 420 aisle); the rails run ~130 mm past each row's last rack to meet the cart, and
    the transfer-rack beam is held from posts beyond the robot's tray end.

Background: [docs/design/001-load-carrying-single-vs-two-rail.md](docs/design/001-load-carrying-single-vs-two-rail.md) (section 8h).
Source: `generator/params_v3.py`, `generator/assembly_v3.py`.
```
pixi run -e sim sim          # v3 in the MuJoCo viewer + task buttons in the browser
pixi run -e sim sim-headless --task 1,4,5   # run tasks without a window, snapshots -> sim/build/snaps_v3/
pixi run export-v3           # v3 STEP/STL -> models/v3/ (new parts + poses pick, swivel, travel, transfer)
pixi run clearance-v3        # collision check, 108 poses (picks, swivels, climb, every row, the cart transfer)
pixi run -e sim legs --v3    # wheel loads, v3 vs v2
pixi run -e sim sim-v2       # the v2 robot (with physics)
```
Tasks: the viewer opens a **task page** in the browser (http://127.0.0.1:8765) with one button per task and a number box
with Enter; it is the only place the tasks are listed. A task number + Enter in the terminal, or keys 6/7/8/9 in the viewer,
also work. `--no-browser` stops the page opening by itself; `--port` changes the port. The sliders in the viewer move the robot
by hand while it is idle (height, hub swivels, crossings, the two picker servos); driving along a row and the cart
transfer are done by the tasks.

**Weight balance, shown live** (static estimate, drawn on a plane just in front of the robot):
* yellow ball = centre of gravity, with a yellow line to the column (the lever arm) and a yellow arrow for the weight;
* white dots = the two hubs;
* blue arrows = sideways rail forces that stop the twist;
* red arrows = in/out rail forces that stop the forward tip.

The bottom-right panel gives the numbers. Gears turn with the motion: the climb pinions roll along the racks; servo 1's pulley, the cross
shaft, the stage-1 idlers and the stage-2 pulleys turn with the arm stroke, and the stage-2 belt runs stretch and shrink;
servo 2, its gear pair and the finger shafts turn with the fingers. The two picker sliders are `servo1_arms` and `servo2_fingers`. Task 11 (*weight-balance demo*) does the following:
1. Pulls the heaviest basket slowly, so the arrows grow as it slides on.
2. Switches the battery's effect off and on, so you can see what the battery box does.

Rail changes vs v2:
* 3 rows of 4 bays joined by a transfer cart at their right ends (v2: 2 bays in a row);
* a second horizontal rail;
* the lower horizontal rail moved from z 140 to z 70;
* four small crossing turntables instead of two 300 mm ones;
* vertical rails 80 mm taller.

Known v3 limits:
* The arms need the ~23 mm gap between the basket and the rack posts.
* Fingers have 8 mm between the basket's far wall and the rack's rear flange (4 mm finger: +-2 mm tip accuracy).
* At full reach the bars overlap only ~29 mm per stage; real bars need side guide lips (so they can't lift off) and stiff
  sections (aluminium 13 x 8 is the minimum; measure the deflection with a 2 kg basket).
* Belts stretch: tension the stage-1 loops and stage-2 belts, and home servo 1 once against the stowed stop.
* Transfer cart: the robot (~3.5 kg + basket) rides 2 short rail pieces; the cart needs a firm floor track, a top
  guide on the rack beam, a detent at each row, and its rail pieces must line up with each row's rails to ~0.2 mm
  (with the racks' tooth phase).
* People share the aisles with the robot: a real system needs a stop sensor or zone lights.
* Basket masses in the sim are demo values.
* Masses and motor ratings are estimates.

### Opening the models
* **FreeCAD 1.0** – open `models/assembly/*.FCStd` or any `.step` (File → Open).
* **Fusion 360** – Upload / Open the `.step` files; every body comes in as a separate component.
* **Slicer** – `models/print/*.stl` (already in print orientation).

## Notes on the exact CAD
* Bought parts (motors, servos, rails, V-wheels, angle iron, baskets) are simplified envelopes.
* A `cyl()` with `seg <= 8` stays a polygon (e.g. the hex nut trap in T06); all other round features are exact.
* All 27 parts are single valid solids; volumes match the original STLs within ~0.5 % (gears 1.7–3 % lower
  because the mesh version double-counted overlapping shells).
* 60T hand gear: the original polyline dipped ~0.8 mm below the root circle; the exact outline starts the
  involute at the root circle (slightly stronger, no functional change).

## Print settings (from the v2 release notes)
PETG (or PLA+), 0.2 mm layers, 4 walls, 40 % gyroid; pinions and racks 100 % infill at 0.12 mm layers.
Motors: climb = JGY-370 1:200 (~22 rpm) with Hall encoder; hand = JGY-370 1:600 (~10 rpm).

## Roadmap
- [x] Exact B-rep export of all parts and assembly poses
- [ ] FreeCAD FEM (CalculiX) studies for the load-bearing parts – `simulation/`
- [ ] Fusion 360 cross-check
