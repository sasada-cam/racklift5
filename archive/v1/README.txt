RackLift 5 - printable parts + parametric source
================================================
stl/            Print-ready STL files, already in print orientation, units = mm.
                *_R / *_L and *_A / *_B are mirror pairs - print the quantity shown in the guide for each.
renders/        Preview images of the assembly.
racklift_assembly_preview.stl   Whole machine in one mesh (for viewing only, do not print).
source/         Parametric Python generator (numpy + Pillow only).
                1. Edit source/params.py (values marked # MEASURE).
                2. python build.py      -> regenerates out/stl, renders and the viewer data.
                3. python clearance.py  -> collision check of 33 lift/arm positions.

Quantities: rack_segment 5R+5L, pinion 2, rail_mount 2A+2B, carriage_bracket 1R+1L, deck_hanger 1R+1L,
wheel_spacer 4, drill_jig 1, shoulder_tower 1, upper_arm 1, forearm_hook 1, basket_stop 2, cam_tof_bracket 1, tof_down_mount 1.
Settings: PETG (or PLA+), 0.2 mm layers, 4 walls, 40% gyroid, pinions 100%.
Supports only under the sensor holder (carriage bracket) and switch shelf (rail mount).
IMPORTANT: use the 30-40 RPM (1:150) or ~22 RPM (1:200) JGY-370 - the 120 RPM one is too weak to lift the tray.
