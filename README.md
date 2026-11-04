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

Without pixi (CAD export only, no FEM solver): `pip install -r requirements.txt` with Python 3.11–3.14.

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
