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

## Setup
```
pip install -r requirements.txt
```
FreeCAD 1.0 is needed for the `.FCStd` files and for FEM. Clone with Git LFS installed (`git lfs install`) –
STL / STEP / FCStd / PNG files are stored in LFS.

## Regenerate after changing `generator/params.py`
```
python cad/export_brep.py                    # all parts + 3 assembly poses -> models/   (~1 min)
python cad/tools/test_parts.py               # volume check of every part vs archive/v2_original_stl
"C:\Program Files\FreeCAD 1.0\bin\freecadcmd.exe" cad/tools/freecad_verify.py   # FreeCAD validity check + .FCStd
```
`python cad/export_brep.py parts` or `... reach` regenerates only that part of the output.
The original mesh pipeline still works: `cd generator && python export_v2.py` (writes `generator/out/`, git-ignored).

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
