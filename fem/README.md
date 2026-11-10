# v3 rail stress test (CalculiX)

`pixi run python fem/rail_stress.py [H324 V694]` -> meshes (gmsh, tet10), solves (ccx), writes `fem/out/` (`results.json`, `*.vtu` for ParaView, `run.log`).

## Model
* Section: the generator's 20x20 V-slot (`vslot_z`), 6063-T5: E 69 GPa, yield 110 MPa. Modelled section is a bare slotted profile
  (Ix = 9.2e3 mm4), softer than stock 2020 extrusion, so results are conservative on stiffness.
* **H324**: the 324 mm horizontal rail piece between two crossings, bolted at both ends (10 mm bolt patches fixed), wheels at midspan (worst).
* **V694**: the 694 mm vertical rail, bolted at the crossing foot and at the two rail bridges (abs. z 650, 955), free top end; upper hub at midspan
  between the bridges, lower hub 259 mm below pushing the opposite way (the twist couple).
* Loads per rail from `sim/legs.py --v3` (2 kg payload, robot 3.5 kg est.): weight share 27 N, twist 24 N at the worst wheel.
  Wheel contact = 8 mm patches on the slot lips. Extra cases: all loads x3 (jolt / e-stop) and a 200 N sideways push at midspan.

## Results
| Case | Rail | Max deflection | Peak von Mises (away from load patches) | Safety factor on yield |
|---|---|---|---|---|
| nominal, 2 kg payload | H324 | 0.010 mm | 2.5 MPa | 44 |
| nominal, 2 kg payload | V694 | 0.009 mm | 2.0 MPa | 56 |
| x3 dynamic | H324 | 0.031 mm | 7.6 MPa | 15 |
| x3 dynamic | V694 | 0.027 mm | 5.9 MPa | 19 |
| 200 N sideways at midspan | H324 | 0.078 mm | 10.9 MPa | 10 |
| 200 N sideways at midspan | V694 | 0.069 mm | 14.6 MPa | 7.5 |

Even counting the stress concentrated under the force patches (not representative of real wheel contact) the worst case keeps SF >= 3.7.
Check: 200 N at midspan of the 324 mm rail gives 0.056 mm (both ends clamped) to 0.111 mm (pinned) by beam theory; FEM 0.078 mm.

## Findings
* The aluminium rails are **not** the limit: deflections are under 0.1 mm and stress under 15 MPa in every case.
* Weak points are elsewhere (not covered by this FEM, hand estimate only): the printed in-slot rack tooth. Lewis, module 1.5, 10 mm face,
  pinion carrying the whole 54 N: ~13 MPa bending vs PETG ~45 MPa along the layers and roughly half across them -> SF ~3.5 nominal, ~1.2 at x3,
  with creep/fatigue still to consider. The M5 T-nut joints, printed rail bridge (PETG), crossing bracket and cart/beam are the next parts to FEM.

## Limits
Static, linear, point-in-time: no wheel flange contact, T-nut / bolt preload, glue joints, dynamic response, or the 489 mm last stretch
(two ~245 mm spans, stiffer than H324). Payload/robot mass are estimates in `params_v3.py` / `legs.py`.
