"""Full Integrated Option 1 Robot + Embedded 2-Stage Forklift Objects Compartment.

Combines the complete Dual-Leg climber + 40x25 column + P07 plugs + H1/H2 pods + P10 hoods +
B01 collar + B03 battery pod + 2-bay 2020 railings & racks (`build_option1_full.py`) WITH
the finalized CNC4759-A 2-Stage Forklift Objects Compartment (`build_objects_compartment.py`),
leaving all legacy files untouched.

Usage:
  .pixi/envs/default/bin/python cad/build_forklift_robot_full.py
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GEN_DIR = os.path.abspath(os.path.join(HERE, "..", "generator"))
sys.path.insert(0, HERE)
sys.path.insert(0, GEN_DIR)

import build123d as bd                      # noqa: E402
import build_t_slot_option1 as O            # noqa: E402
import build_option1_dual_legs as DL        # noqa: E402
import build_option1_body as OB             # noqa: E402
import build_option1_full as OF             # noqa: E402
import build_objects_compartment as OC      # noqa: E402

OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "models", "components", "forklift_robot_full"))
SIM_MESH_DIR = os.path.abspath(os.path.join(HERE, "..", "sim", "build", "meshes_forklift_robot_full"))


def _overlap(a, b) -> float:
    return OC._overlap(a, b)


def check_full_forklift_clearances(
    col, plugs, hood_up, hood_lo, env, c_mags, s_fix, s_turn, s_hw,
    robot_parts: dict, forklift_parts: dict,
) -> bool:
    """Verify 0 collisions across whole robot + embedded CNC4759-A Forklift across all poses."""
    pod_parts = O.build_assembled_parts()
    fixed_keys = ("body_mount", "bearing_bot", "bearing_top", "m8_hardware", "servo_body",
                  "fasteners_fixed", "gear_20t", "alum_horn", "servo_spline", "fasteners_gear")
    turn_keys = ("gantry_plate", "v_wheels", "motor_housing", "motor_shaft", "pinion", "retainer",
                 "bearing_pinion", "carriage_pod", "motor_cap", "pinion_cover", "fasteners_turn",
                 "screws_pcover", "screws_cap", "screw_pinion")

    def pre_boxes(shape):
        sols = shape.solids() if hasattr(shape, "solids") else [shape]
        return [(s, s.bounding_box()) for s in sols]

    def overlap_pre(a, b_list):
        ba = a.bounding_box()
        tot = 0.0
        for sb, bb in b_list:
            if not ba.overlaps(bb):
                continue
            inter = a.intersect(sb)
            if inter is not None and hasattr(inter, "solids"):
                tot += sum(s.volume for s in inter.solids())
        return tot

    env_boxes = {k: pre_boxes(env[k]) for k in (
        "rails_vert", "racks_vert", "rails_horiz", "racks_horiz",
        "crossings_fixed", "rack_bays", "rack_mounts", "rack_mount_hw"
    )}
    mag_boxes = pre_boxes(c_mags)
    turning_tpl = env["turning_tpl"]

    static_fk = (
        "compartment_deck", "fork_under_tie", "fork_top_cover", "fork_cover_fasteners",
        "tray_deck", "fork_base_rails", "fork_front_arch", "fork_drive", "fork_limit_switches",
        "fork_hold_rails", "fork_hold_fasteners",
        "base_boom", "load_cell", "live_wishbone",
    )
    move_body_keys = ("body_collar", "battery_pod", "battery_lid", "battery_pack", "body_fasteners")

    # Poses: (label, dx, dz, rot_y, fork_ext_y)
    poses = (
        ("bay0_home_vertical",         0.0,            0.0,   0.0,   0.0),
        ("bay0_fork_reach_lifted_10",  0.0,           10.0,   0.0, 240.0),
        ("bay0_swivel_45",             0.0,            0.0, -45.0,   0.0),
        ("bay0_swivel_90",             0.0,            0.0, -90.0,   0.0),
        ("bay1_horiz_travel",          OF.BAY_P_FULL,  0.0, -90.0,   0.0),
    )

    bad = []
    for label, dx, dz, rot_y, ext_y in poses:
        shift = bd.Location((dx, 0.0, dz))
        col_m = col.moved(shift)
        plugs_m = plugs.moved(shift)
        hoods = (hood_up.moved(shift), hood_lo.moved(shift))

        # Moved robot body + embedded Forklift compartment
        b_world = {k: robot_parts[k].moved(shift) for k in move_body_keys}
        for k in static_fk:
            b_world[k] = forklift_parts[k].moved(shift)
        for k in ("fork_mid_stage", "fork_t8_nut", "fork_mid_fasteners"):
            b_world[k] = forklift_parts[k].moved(bd.Location((dx, 0.5 * ext_y, dz)))
        rot_deg = math.degrees((0.5 * ext_y) / OC.PIN_RP)
        for pk, sgn, ang in (("fork_pinion_left", -1, -rot_deg), ("fork_pinion_right", 1, rot_deg)):
            xp = OB.BXR + sgn * (OC.FORK_DX - OC.PIN_DX_CENTER)
            pr = forklift_parts[pk].moved(bd.Location((-xp, -OC.PIN_Y_HOME, 0.0)))
            pr = pr.moved(bd.Location((0.0, 0.0, 0.0), (0.0, 0.0, ang)))
            b_world[pk] = pr.moved(bd.Location((dx + xp, OC.PIN_Y_HOME + 0.5 * ext_y, dz)))
        b_world["fork_tines"] = forklift_parts["fork_tines"].moved(bd.Location((dx, ext_y, dz)))

        # Legs at JZ_LO & JZ_UP
        hubs = {}
        for tag, zh in (("upper", DL.JZ_UP), ("lower", DL.JZ_LO)):
            loc_fix = DL.hub_world_loc(zh + dz, DL.HUB_BASE[tag], dx)
            loc_turn = DL.hub_world_loc(zh + dz, DL.HUB_BASE[tag] + rot_y, dx)
            for k in fixed_keys:
                hubs[f"{tag}_{k}"] = DL.to_hub(pod_parts[k], loc_fix)
            for k in turn_keys:
                hubs[f"{tag}_{k}"] = DL.to_hub(pod_parts[k], loc_turn)
            hubs[f"{tag}_s_fix"] = DL.to_hub(s_fix, loc_fix)
            hubs[f"{tag}_s_turn"] = DL.to_hub(s_turn, loc_turn)
            hubs[f"{tag}_s_hw"] = DL.to_hub(s_hw, loc_turn)

        # Crossing turntables
        turn_cross_solids = []
        for bx in OF.BAY_XS_FULL:
            ang = rot_y if (abs(dx - bx) < 0.5 and abs(dz) < 0.5) else 0.0
            for zc in (DL.JZ_LO, DL.JZ_UP):
                loc_c = DL.hub_world_loc(zc, ang, bx) * DL.LOC_POD_TO_HUB
                for tk in ("xp_rotor", "xp_pivot_608zz", "xp_pivot_m8_stack", "xp_rotor_screws", "xp_stop_pin", "xp_tnuts"):
                    turn_cross_solids.append(turning_tpl[tk].moved(loc_c))
        turn_cross_cmp = bd.Compound(children=turn_cross_solids)

        # Check all Forklift + Body parts vs Hoods, Column, H1/H2 Pods, Magnets, Rails & Racks
        check_keys = ("compartment_deck", "fork_under_tie", "fork_top_cover", "fork_cover_fasteners", "tray_deck", "fork_base_rails",
                      "fork_front_arch", "fork_limit_switches", "fork_hold_rails", "fork_hold_fasteners",
                      "fork_mid_stage", "fork_pinion_left", "fork_pinion_right", "fork_t8_nut", "fork_mid_fasteners", "fork_tines",
                      "fork_drive", "base_boom", "live_wishbone", "load_cell", "body_collar", "battery_pod")
        for bk in check_keys:
            bs = b_world[bk]
            for h_idx, h_s in enumerate(hoods):
                v = _overlap(bs, h_s)
                if v > 1e-3:
                    bad.append((label, bk, f"hood_{h_idx}", v))
            if bk != "body_collar":
                for sc_k, sc_s in (("column_bar", col_m), ("plugs", plugs_m)):
                    v = _overlap(bs, sc_s)
                    if v > 1e-3:
                        bad.append((label, bk, sc_k, v))
            for hk, hs in hubs.items():
                v = _overlap(bs, hs)
                if v > 1e-3:
                    bad.append((label, bk, hk, v))
            v_mag = overlap_pre(bs, mag_boxes)
            if v_mag > 1e-3:
                bad.append((label, bk, "crossing_magnets", v_mag))
            for ek in env_boxes:
                v = overlap_pre(bs, env_boxes[ek])
                if v > 1e-3:
                    bad.append((label, bk, ek, v))
            v_tc = _overlap(bs, turn_cross_cmp)
            if v_tc > 1e-3:
                bad.append((label, bk, "crossings_turn", v_tc))

        print(f"[{label:28s}] collisions so far: {len(bad)}")

    if bad:
        for item in bad:
            print("  COLLISION:", item)
    else:
        print("RESULT: OK - 0 collisions across Full Robot (H1/H2 legs + column + B01/B03 + embedded Forklift + rails + 2-bay racks)")
    return len(bad) == 0


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(SIM_MESH_DIR, exist_ok=True)

    col, plugs = OF.build_unified_column_and_plugs()
    s_fix, s_turn, s_hw, _, _ = DL.build_sensors_and_magnets()
    env, c_mags = OF.build_compact_railing_and_magnets()
    p10_cover, p11_cover, hood_hw = OF.build_leg_protective_covers()

    collar, fast_col = OB.build_body_column_collar()
    bat_box, bat_lid, bat_pack = OB.build_battery_pod()
    forklift_parts = OC.build_objects_base_and_forklift()

    # Merge collar fasteners + loadcell/compartment fasteners cleanly (drop ref_rack_bay from Stage 14 since env has full 2-bay racks)
    fast_all = bd.Compound(
        children=[*fast_col.solids(), *hood_hw.solids(), *forklift_parts["base_fasteners"].solids()],
        label="Robot_Body_Hood_Forklift_Fasteners",
    )

    robot_parts = {
        "body_collar": collar,
        "body_fasteners": fast_all,
        "battery_pod": bat_box,
        "battery_lid": bat_lid,
        "battery_pack": bat_pack,
    }

    export_meshes = {
        "column_bar": col,
        "anti_crush_inserts": plugs,
        "sensors_pod_fixed": s_fix,
        "sensors_pod_turn": s_turn,
        "sensors_hardware": s_hw,
        "leg_hood_upper": p10_cover,
        "leg_hood_lower": p11_cover,
        "crossing_magnets": c_mags,
        "rails_vert": env["rails_vert"],
        "racks_vert": env["racks_vert"],
        "rails_horiz": env["rails_horiz"],
        "racks_horiz": env["racks_horiz"],
        "crossings_fixed": env["crossings_fixed"],
        "rack_bays": env["rack_bays"],
        "rack_mounts": env["rack_mounts"],
        "rack_mount_hw": env["rack_mount_hw"],
        **robot_parts,
        "tray_boom": forklift_parts["base_boom"],
        "load_cell": forklift_parts["load_cell"],
        "live_wishbone": forklift_parts["live_wishbone"],
        "tray_deck": forklift_parts["tray_deck"],
        "compartment_deck": forklift_parts["compartment_deck"],
        "fork_under_tie": forklift_parts["fork_under_tie"],
        "fork_top_cover": forklift_parts["fork_top_cover"],
        "fork_cover_fasteners": forklift_parts["fork_cover_fasteners"],
        "fork_limit_switches": forklift_parts["fork_limit_switches"],
        "fork_hold_rails": forklift_parts["fork_hold_rails"],
        "fork_hold_fasteners": forklift_parts["fork_hold_fasteners"],
        "fork_base_rails": forklift_parts["fork_base_rails"],
        "fork_front_arch": forklift_parts["fork_front_arch"],
        "fork_mid_stage": forklift_parts["fork_mid_stage"],
        "fork_pinion_left": forklift_parts["fork_pinion_left"],
        "fork_pinion_right": forklift_parts["fork_pinion_right"],
        "fork_t8_nut": forklift_parts["fork_t8_nut"],
        "fork_mid_fasteners": forklift_parts["fork_mid_fasteners"],
        "fork_tines": forklift_parts["fork_tines"],
        "fork_drive": forklift_parts["fork_drive"],
        "ref_load": forklift_parts["ref_load"],
    }
    for name, shape in export_meshes.items():
        bd.export_stl(shape, os.path.join(SIM_MESH_DIR, f"{name}.stl"), tolerance=0.05, angular_tolerance=0.2)
    print(f"Exported {len(export_meshes)} full-robot + Forklift viewer meshes to {SIM_MESH_DIR}")

    ok = check_full_forklift_clearances(
        col, plugs, p10_cover, p11_cover, env, c_mags, s_fix, s_turn, s_hw,
        robot_parts, forklift_parts,
    )
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
