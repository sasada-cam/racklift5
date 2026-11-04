RackLift 5 - v2 (rail grid + turntable junctions)
=================================================
stl/      Print-ready STL files, already in print orientation (mm). Quantities are in parts_list.csv.
          R = robot, T = tray / hook arm, J = junction (turntable), K = in-slot gear racks.
renders/  Preview images.
source/   Parametric Python generator (numpy + Pillow only).
          params.py      all dimensions (values marked # MEASURE: check with a caliper)
          export_v2.py   -> out/stl
          clearance_v2.py -> collision check of 67 robot / turntable positions
          r2.py          -> renders

Key changes from v1: one climbing robot instead of a 2-rail lift; gear racks sit INSIDE the rail's front slot;
2 x 300 mm turntable rail pieces at the bottom junctions; the robot's hand (2nd worm motor, 12T->60T) pushes on two
fixed pins to turn itself + the rail piece 90 deg while the tray stays level.
Motors: climb = JGY-370 1:200 (~22 rpm) WITH Hall encoder; hand = JGY-370 1:600 (~10 rpm).
