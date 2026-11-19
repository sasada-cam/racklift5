"""Generate ISO A3 2D Engineering Drawings (SVG + DXF) with Build Process & Dimensions.

Uses build123d OpenCASCADE Hidden-Line Removal (`project_to_viewport`), `ExportDXF`,
and automated B-rep bounding/bore extraction to produce dimensioned 3-view + isometric
engineering sheets in `docs/drawings/` for:
  - All Forklift Objects Compartment printed parts (R44-FK, R46..R55)
  - All Option 1 Robot Body printed parts (R40..R45, P07B)
  - All Swivel Pod, Sensor & Hood/Crossing parts (P01..P11, crossings)
  - All Workshop Lathe / CNC / Shop-Cut metal parts (M01..M04, T8 cut, smooth shafts, column)

Usage:
  .pixi/envs/default/bin/python cad/export_drawings.py
"""

from collections import Counter
import html
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
OUT_SVG_DIR = os.path.join(ROOT, "docs", "drawings")
OUT_DXF_DIR = os.path.join(OUT_SVG_DIR, "dxf")
sys.path.insert(0, HERE)

import build123d as bd  # noqa: E402
import build_t_slot_option1 as O1  # noqa: E402
import build_option1_dual_legs as DL  # noqa: E402
import build_option1_body as OB  # noqa: E402
import build_option1_full as OF  # noqa: E402
import build_objects_compartment as OC  # noqa: E402
import build_rail_racks_option1 as RR  # noqa: E402


BUILD_STYLE = {
    "3D PRINT (FDM)": ("#059669", "#ecfdf5"),
    "LATHE TURNING": ("#d97706", "#fffbeb"),
    "CNC / 3D PRINT": ("#2563eb", "#eff6ff"),
    "SHOP CUT / DRILL": ("#7c3aed", "#f5f3ff"),
}

STD_SCALES = [4.0, 3.0, 2.0, 1.5, 1.0, 0.75, 0.5, 0.4, 0.3, 0.25, 0.2]


def _scale_label(s: float) -> str:
    if abs(s - round(s)) < 1e-6 and s >= 1.0:
        return f"{int(round(s))}:1"
    inv = 1.0 / s
    if abs(inv - round(inv)) < 0.05:
        return f"1:{int(round(inv))}"
    return f"{s:.2f}:1"


def _sample_edges_to_svg_paths(edges, cx_svg: float, cy_svg: float, scale: float, tol_arc: float = 0.02) -> list[str]:
    paths = []
    for e in edges:
        try:
            gtype = e.geom_type
        except Exception:
            gtype = None
        if gtype == bd.GeomType.LINE:
            p0 = e.position_at(0.0)
            p1 = e.position_at(1.0)
            x0, y0 = cx_svg + p0.X * scale, cy_svg - p0.Y * scale
            x1, y1 = cx_svg + p1.X * scale, cy_svg - p1.Y * scale
            if (x1 - x0) ** 2 + (y1 - y0) ** 2 > 1e-4:
                paths.append(f"M{x0:.2f},{y0:.2f}L{x1:.2f},{y1:.2f}")
        else:
            n = 28 if gtype == bd.GeomType.CIRCLE else 18
            pts = []
            for i in range(n + 1):
                p = e.position_at(i / n)
                pts.append((cx_svg + p.X * scale, cy_svg - p.Y * scale))
            if len(pts) >= 2:
                seg = [f"M{pts[0][0]:.2f},{pts[0][1]:.2f}"]
                seg.extend(f"L{x:.2f},{y:.2f}" for x, y in pts[1:])
                paths.append("".join(seg))
    return paths


def _extract_circle_callouts(shape: bd.Shape) -> list[str]:
    counts: Counter = Counter()
    try:
        edges = shape.edges()
    except Exception:
        return []
    for e in edges:
        try:
            if e.geom_type == bd.GeomType.CIRCLE and getattr(e, "radius", 0.0) >= 0.45:
                d = round(2.0 * e.radius, 2)
                if 1.0 <= d <= 60.0:
                    # Group near-identical diameters (e.g. 2x edges per through-hole -> /2 approx)
                    key = round(d * 20.0) / 20.0
                    counts[key] += 1
        except Exception:
            continue
    if not counts:
        return []
    # Deduplicate end-of-cylinder pairs (divide edge count cleanly for display summary)
    summary = []
    for diam, cnt in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))[:6]:
        holes = max(1, (cnt + 1) // 2)
        lbl = f"Ø{diam:.2f} mm" + (f" (×{holes})" if holes > 1 else "")
        summary.append(lbl)
    return summary


def _dim_h(x0: float, x1: float, y: float, y_ext0: float, text: str) -> str:
    if x1 < x0:
        x0, x1 = x1, x0
    xm = 0.5 * (x0 + x1)
    a = 2.6
    return (
        f'<g class="dim">'
        f'<line x1="{x0:.2f}" y1="{y_ext0:.2f}" x2="{x0:.2f}" y2="{y + (3 if y > y_ext0 else -3):.2f}"/>'
        f'<line x1="{x1:.2f}" y1="{y_ext0:.2f}" x2="{x1:.2f}" y2="{y + (3 if y > y_ext0 else -3):.2f}"/>'
        f'<line x1="{x0:.2f}" y1="{y:.2f}" x2="{x1:.2f}" y2="{y:.2f}"/>'
        f'<polygon points="{x0:.2f},{y:.2f} {x0+a:.2f},{y-1.1:.2f} {x0+a:.2f},{y+1.1:.2f}"/>'
        f'<polygon points="{x1:.2f},{y:.2f} {x1-a:.2f},{y-1.1:.2f} {x1-a:.2f},{y+1.1:.2f}"/>'
        f'<rect x="{xm - len(text)*1.75 - 2:.2f}" y="{y - 3.6:.2f}" width="{len(text)*3.5 + 4:.2f}" height="6.2" fill="#ffffff"/>'
        f'<text x="{xm:.2f}" y="{y + 0.8:.2f}" text-anchor="middle">{html.escape(text)}</text>'
        f'</g>'
    )


def _dim_v(x: float, y0: float, y1: float, x_ext0: float, text: str) -> str:
    if y1 < y0:
        y0, y1 = y1, y0
    ym = 0.5 * (y0 + y1)
    a = 2.6
    return (
        f'<g class="dim">'
        f'<line x1="{x_ext0:.2f}" y1="{y0:.2f}" x2="{x + (3 if x > x_ext0 else -3):.2f}" y2="{y0:.2f}"/>'
        f'<line x1="{x_ext0:.2f}" y1="{y1:.2f}" x2="{x + (3 if x > x_ext0 else -3):.2f}" y2="{y1:.2f}"/>'
        f'<line x1="{x:.2f}" y1="{y0:.2f}" x2="{x:.2f}" y2="{y1:.2f}"/>'
        f'<polygon points="{x:.2f},{y0:.2f} {x-1.1:.2f},{y0+a:.2f} {x+1.1:.2f},{y0+a:.2f}"/>'
        f'<polygon points="{x:.2f},{y1:.2f} {x-1.1:.2f},{y1-a:.2f} {x+1.1:.2f},{y1-a:.2f}"/>'
        f'<g transform="translate({x - 1.5:.2f},{ym:.2f}) rotate(-90)">'
        f'<rect x="{-len(text)*1.75 - 2:.2f}" y="-4.8" width="{len(text)*3.5 + 4:.2f}" height="6.2" fill="#ffffff"/>'
        f'<text x="0" y="-0.4" text-anchor="middle">{html.escape(text)}</text>'
        f'</g></g>'
    )


def _bounds_2d(edges) -> tuple[float, float, float, float]:
    xs, ys = [], []
    for e in edges:
        bb = e.bounding_box()
        xs.extend((bb.min.X, bb.max.X))
        ys.extend((bb.min.Y, bb.max.Y))
    return (min(xs), max(xs), min(ys), max(ys)) if xs else (0.0, 1.0, 0.0, 1.0)


def generate_drawing(spec: dict) -> dict:
    """Generate A3 4-view dimensioned SVG + 2D DXF for one part."""
    pid = spec["id"]
    raw = spec["shape"]
    bb = raw.bounding_box()
    ctr = bb.center()
    sh = raw.moved(bd.Location((-ctr.X, -ctr.Y, -ctr.Z)))
    dx, dy, dz = bb.size.X, bb.size.Y, bb.size.Z
    vol_cm3 = (sh.volume / 1000.0) if hasattr(sh, "volume") else 0.0

    # Pick largest standard scale fitting Top (X x Y), Front (X x Z), Side (Y x Z) in A3 zones
    ortho_scale = 0.2
    for s in STD_SCALES:
        fits_top = (dx * s <= 156.0) and (dy * s <= 94.0)
        fits_front = (dx * s <= 156.0) and (dz * s <= 76.0)
        fits_side = (dy * s <= 92.0) and (dz * s <= 76.0)
        if fits_top and fits_front and fits_side:
            ortho_scale = s
            break

    # View centers on 420 x 297 A3 sheet
    top_cx, top_cy = 104.0, 74.0
    front_cx, front_cy = 104.0, 188.0
    side_cx, side_cy = 236.0, 188.0
    iso_cx, iso_cy = 346.0, 80.0

    dist = max(250.0, max(dx, dy, dz) * 3.0)
    top_vis, top_hid = sh.project_to_viewport((0, 0, dist), (0, 1, 0), look_at=(0, 0, 0))
    front_vis, front_hid = sh.project_to_viewport((0, -dist, 0), (0, 0, 1), look_at=(0, 0, 0))
    side_vis, side_hid = sh.project_to_viewport((dist, 0, 0), (0, 0, 1), look_at=(0, 0, 0))
    iso_vis, iso_hid = sh.project_to_viewport((dist, -dist, 0.78 * dist), (0, 0, 1), look_at=(0, 0, 0))

    ix0, ix1, iy0, iy1 = _bounds_2d([*iso_vis, *iso_hid])
    iso_w, iso_h = max(1e-3, ix1 - ix0), max(1e-3, iy1 - iy0)
    iso_scale = min(104.0 / iso_w, 108.0 / iso_h)

    # Export clean 2D DXF (Front/Top/Side laid out cleanly at 1:1 mm)
    dxf_path = os.path.join(OUT_DXF_DIR, f"{pid}.dxf")
    try:
        dxf = bd.ExportDXF()
        dxf.add_layer("VISIBLE", color=bd.ColorIndex.BLACK, line_weight=0.35)
        dxf.add_layer("HIDDEN", color=bd.ColorIndex.BLUE, line_weight=0.18, line_type=bd.LineType.HIDDEN)
        gap = 25.0
        for e in top_vis:
            dxf.add_shape(e.moved(bd.Location((0, 0.5 * dz + gap + 0.5 * dy, 0))), layer="VISIBLE")
        for e in top_hid:
            dxf.add_shape(e.moved(bd.Location((0, 0.5 * dz + gap + 0.5 * dy, 0))), layer="HIDDEN")
        dxf.add_shape(front_vis, layer="VISIBLE")
        dxf.add_shape(front_hid, layer="HIDDEN")
        for e in side_vis:
            dxf.add_shape(e.moved(bd.Location((0.5 * dx + gap + 0.5 * dy, 0, 0))), layer="VISIBLE")
        for e in side_hid:
            dxf.add_shape(e.moved(bd.Location((0.5 * dx + gap + 0.5 * dy, 0, 0))), layer="HIDDEN")
        dxf.write(dxf_path)
    except Exception:
        dxf_path = None

    # Render crisp SVG paths
    vis_paths = (
        _sample_edges_to_svg_paths(top_vis, top_cx, top_cy, ortho_scale)
        + _sample_edges_to_svg_paths(front_vis, front_cx, front_cy, ortho_scale)
        + _sample_edges_to_svg_paths(side_vis, side_cx, side_cy, ortho_scale)
        + _sample_edges_to_svg_paths(iso_vis, iso_cx - 0.5 * (ix0 + ix1) * iso_scale, iso_cy + 0.5 * (iy0 + iy1) * iso_scale, iso_scale)
    )
    hid_paths = (
        _sample_edges_to_svg_paths(top_hid, top_cx, top_cy, ortho_scale)
        + _sample_edges_to_svg_paths(front_hid, front_cx, front_cy, ortho_scale)
        + _sample_edges_to_svg_paths(side_hid, side_cx, side_cy, ortho_scale)
        + _sample_edges_to_svg_paths(iso_hid, iso_cx - 0.5 * (ix0 + ix1) * iso_scale, iso_cy + 0.5 * (iy0 + iy1) * iso_scale, iso_scale)
    )

    # Exact projected view bounding boxes for accurate dimension lines
    tx0, tx1, ty0, ty1 = _bounds_2d(top_vis)
    fx0, fx1, fz0, fz1 = _bounds_2d(front_vis)
    sy0, sy1, sz0, sz1 = _bounds_2d(side_vis)

    dims_svg = "".join([
        _dim_h(top_cx + tx0 * ortho_scale, top_cx + tx1 * ortho_scale, top_cy - ty1 * ortho_scale - 9.0, top_cy - ty1 * ortho_scale, f"L = {dx:.2f} mm"),
        _dim_v(top_cx + tx0 * ortho_scale - 10.0, top_cy - ty1 * ortho_scale, top_cy - ty0 * ortho_scale, top_cx + tx0 * ortho_scale, f"W = {dy:.2f} mm"),
        _dim_h(front_cx + fx0 * ortho_scale, front_cx + fx1 * ortho_scale, front_cy - fz0 * ortho_scale + 9.5, front_cy - fz0 * ortho_scale, f"{dx:.2f} mm"),
        _dim_v(front_cx + fx0 * ortho_scale - 10.0, front_cy - fz1 * ortho_scale, front_cy - fz0 * ortho_scale, front_cx + fx0 * ortho_scale, f"H = {dz:.2f} mm"),
        _dim_h(side_cx + sy0 * ortho_scale, side_cx + sy1 * ortho_scale, side_cy - sz0 * ortho_scale + 9.5, side_cy - sz0 * ortho_scale, f"{dy:.2f} mm"),
    ])

    holes = _extract_circle_callouts(sh)
    holes_str = ", ".join(holes) if holes else "None / flat profiles"
    proc = spec["process"]
    fg_p, bg_p = BUILD_STYLE.get(proc, ("#334155", "#f1f5f9"))
    mark_line = (
        f"Engrave (0.6 mm): {spec['mark']}"
        if (proc == "3D PRINT (FDM)" and spec["id"] not in ("R44-FK", "R44-PK"))
        else f"Part Mark / Stamp: {spec['mark']}"
    )
    dims_list = spec.get("key_dims", [])
    dims_html = "".join(
        f'<text x="291" y="{169 + idx * 6.5:.1f}" class="sm">• {html.escape(line)}</text>'
        for idx, line in enumerate(dims_list[:5])
    )
    notes_lines = [
        f"Critical Features / Callouts: {holes_str}",
        f"Process & Settings: {spec['material']} — {spec['notes']}",
        f"Source File: {spec['rel_path']}",
    ]
    notes_html = "".join(
        f'<text x="15" y="{261 + idx * 7.5:.1f}" class="sm">{html.escape(line)}</text>'
        for idx, line in enumerate(notes_lines)
    )

    svg_text = f"""<?xml version="1.0" encoding="utf-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="420mm" height="297mm" viewBox="0 0 420 297">
  <style>
    text {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, sans-serif; fill: #0f172a; }}
    .vis {{ fill: none; stroke: #0f172a; stroke-width: 0.42; stroke-linecap: round; stroke-linejoin: round; }}
    .hid {{ fill: none; stroke: #94a3b8; stroke-width: 0.22; stroke-dasharray: 1.4,1.1; stroke-linecap: round; }}
    .dim line {{ stroke: #0284c7; stroke-width: 0.25; }}
    .dim polygon {{ fill: #0284c7; }}
    .dim text {{ fill: #0369a1; font-size: 4.0px; font-weight: 700; }}
    .frame {{ fill: none; stroke: #1e293b; stroke-width: 0.5; }}
    .grid {{ fill: none; stroke: #cbd5e1; stroke-width: 0.2; }}
    .lbl {{ font-size: 3.8px; font-weight: 700; fill: #475569; letter-spacing: 0.4px; }}
    .sm {{ font-size: 3.4px; fill: #334155; }}
    .ttl {{ font-size: 5.5px; font-weight: 800; fill: #0f172a; }}
    .sub {{ font-size: 3.8px; font-weight: 600; fill: #475569; }}
  </style>
  <rect x="0" y="0" width="420" height="297" fill="#ffffff"/>
  <!-- ISO A3 Border & View Dividers -->
  <rect x="10" y="10" width="400" height="277" class="frame"/>
  <line x1="286" y1="10" x2="286" y2="287" class="frame"/>
  <line x1="10" y1="251" x2="286" y2="251" class="frame"/>
  <line x1="286" y1="150" x2="410" y2="150" class="frame"/>
  <line x1="286" y1="215" x2="410" y2="215" class="frame"/>
  <line x1="188" y1="132" x2="188" y2="251" class="grid"/>
  <line x1="10" y1="132" x2="286" y2="132" class="grid"/>

  <!-- View Labels -->
  <text x="15" y="17" class="lbl">TOP VIEW (X × Y)</text>
  <text x="15" y="139" class="lbl">FRONT VIEW (X × Z)</text>
  <text x="193" y="139" class="lbl">RIGHT VIEW (Y × Z)</text>
  <text x="291" y="17" class="lbl">ISOMETRIC 3D</text>

  <!-- Hidden & Visible Projected CAD Geometry -->
  <path class="hid" d="{' '.join(hid_paths)}"/>
  <path class="vis" d="{' '.join(vis_paths)}"/>

  <!-- Dimension Callouts -->
  {dims_svg}

  <!-- Right-Mid Specifications Panel -->
  <text x="291" y="157" class="lbl">CRITICAL DIMENSIONS &amp; FIT</text>
  <text x="291" y="162.5" class="sm">Envelope: {dx:.2f} × {dy:.2f} × {dz:.2f} mm ({vol_cm3:.2f} cm³)</text>
  {dims_html}

  <!-- Bottom Notes Strip -->
  <text x="15" y="256" class="lbl">FABRICATION &amp; ASSEMBLY NOTES</text>
  {notes_html}

  <!-- ISO Title Block -->
  <rect x="290" y="219" width="116" height="11" rx="2" fill="{bg_p}" stroke="{fg_p}" stroke-width="0.45"/>
  <text x="348" y="226" text-anchor="middle" style="font-size:4.6px;font-weight:800;fill:{fg_p}">{html.escape(proc)}</text>
  <text x="291" y="238" class="ttl">{html.escape(spec['mark'])} · {html.escape(spec['short_title'])}</text>
  <text x="291" y="244" class="sub">{html.escape(spec['name'][:52])}</text>
  <line x1="286" y1="249" x2="410" y2="249" class="grid"/>
  <text x="291" y="255" class="sm">Qty / Robot: {html.escape(str(spec['qty']))}</text>
  <text x="352" y="255" class="sm">Scale: {_scale_label(ortho_scale)}</text>
  <text x="291" y="262" class="sm">Material: {html.escape(spec['material'][:54])}</text>
  <text x="291" y="269" class="sm">Projection: ISO 3rd Angle · Units: mm</text>
  <text x="291" y="276" class="sm">Project: RackLift5 · Embedded Forklift</text>
  <text x="291" y="283" class="sm">{html.escape(mark_line)}</text>
</svg>
"""
    svg_path = os.path.join(OUT_SVG_DIR, f"{pid}.svg")
    with open(svg_path, "w", encoding="utf-8") as f:
        f.write(svg_text)

    return {
        "id": pid,
        "mark": spec["mark"],
        "process": proc,
        "bbox_mm": [round(dx, 2), round(dy, 2), round(dz, 2)],
        "volume_cm3": round(vol_cm3, 2),
        "holes": holes,
        "svg_rel": os.path.relpath(svg_path, ROOT).replace(os.sep, "/"),
        "dxf_rel": os.path.relpath(dxf_path, ROOT).replace(os.sep, "/") if dxf_path else None,
        "scale": _scale_label(ortho_scale),
    }


def _build_metal_shop_shapes() -> dict:
    """Build exact B-rep models for workshop-lathed & shop-cut metal parts."""
    # M01: Bright mild/silver steel Ø8 x 69 mm swivel axle drilled Ø3.3 x 12 & M4 tapped both ends
    m01 = bd.Cylinder(radius=4.0, height=69.0) - bd.Cylinder(radius=1.65, height=12.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX)).moved(bd.Location((0, 0, 34.5))) - bd.Cylinder(radius=1.65, height=12.0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((0, 0, -34.5)))
    m01 = m01.moved(bd.Location((0, 0, 0), (0, 90, 0)))  # orient horizontally along X so top/front views show length
    # M02, M03, M04: Steel axle spacer tubes (12 OD x 8.2 ID)
    m02 = (bd.Cylinder(radius=6.0, height=12.0) - bd.Cylinder(radius=4.1, height=13.0)).moved(bd.Location((0, 0, 0), (0, 90, 0)))
    m03 = (bd.Cylinder(radius=6.0, height=4.5) - bd.Cylinder(radius=4.1, height=5.5)).moved(bd.Location((0, 0, 0), (0, 90, 0)))
    m04 = (bd.Cylinder(radius=6.0, height=1.5) - bd.Cylinder(radius=4.1, height=2.5)).moved(bd.Location((0, 0, 0), (0, 90, 0)))
    # M11: Crossing 608ZZ Pivot Bearing Housing (6061-T6 Alu Lathe Turned)
    m11 = bd.import_step(os.path.join(ROOT, "models/components/t_slot_option1_crossing/fx_pivot_housing.step"))
    # M13, M14, M15: Crossing axle steel spacer tubes (12 OD x 8.2 ID) & M16 stop-pin sleeve (10 OD x 5.2 ID)
    m13 = (bd.Cylinder(radius=6.0, height=2.5) - bd.Cylinder(radius=4.1, height=3.5)).moved(bd.Location((0, 0, 0), (0, 90, 0)))
    m14 = (bd.Cylinder(radius=6.0, height=7.95) - bd.Cylinder(radius=4.1, height=8.95)).moved(bd.Location((0, 0, 0), (0, 90, 0)))
    m15 = (bd.Cylinder(radius=6.0, height=2.0) - bd.Cylinder(radius=4.1, height=3.0)).moved(bd.Location((0, 0, 0), (0, 90, 0)))
    m16 = (bd.Cylinder(radius=5.0, height=20.0) - bd.Cylinder(radius=2.6, height=21.0)).moved(bd.Location((0, 0, 0), (0, 90, 0)))
    # T8 Cut Lead Screw (Ø8 x 202 mm 304SS) & MT0092 Smooth Guide Rod (Ø8 x 165 mm)
    m_t8 = OC._build_t8_threaded_screw(0.0, 202.0, 0.0, 0.0).moved(bd.Location((0, 0, 0), (0, 0, 90)))
    m_rod = bd.Cylinder(radius=4.0, height=165.0).moved(bd.Location((0, 0, 0), (0, 90, 0)))
    # 40 x 25 x 2 mm Aluminum Column (276 mm)
    m_col, _ = OF.build_unified_column_and_plugs()
    m_col = m_col.moved(bd.Location((0, 0, 0), (0, 90, 0)))
    return {"M01": m01, "M02": m02, "M03": m03, "M04": m04, "M11": m11, "M13": m13, "M14": m14, "M15": m15, "M16": m16, "M-T8": m_t8, "M-ROD": m_rod, "M-COL": m_col}


def build_all_specs() -> list[dict]:
    fork = OC.build_objects_base_and_forklift()
    collar, _ = OB.build_body_column_collar()
    bat_box, bat_lid, _ = OB.build_battery_pod()
    boom, live_wish, _, _ = OB.build_tray_boom_and_loadcell()
    picker_orig = OB.build_tray_and_picker()
    _, col_plug = OB.build_column_stub_and_plug()
    o1 = O1.build_assembled_parts()
    s_fix, _, _, _, p08_bkt = DL.build_sensors_and_magnets()
    p09_raw = s_fix.solids()[0]
    bb09 = p09_raw.bounding_box()
    p09_bkt = p09_raw.moved(bd.Location((-0.5 * (bb09.min.X + bb09.max.X), -0.5 * (bb09.min.Y + bb09.max.Y), -bb09.min.Z)))
    bd.export_stl(p09_bkt, os.path.join(ROOT, "models/components/option1_dual_legs/opt1_P09_swivel_sensor_bridge.stl"), tolerance=0.01, angular_tolerance=0.1)
    _, col_end_plugs = DL.build_anti_crush_inserts()
    p10_up_raw, p10_lo_raw, _ = OF.build_leg_protective_covers()
    metal = _build_metal_shop_shapes()

    xp_rotor = bd.import_step(os.path.join(ROOT, "models/components/t_slot_option1_crossing/xp_rotor.step"))
    xp_rack = bd.import_step(os.path.join(ROOT, "models/components/t_slot_option1_crossing/xp_rack.step"))
    fx_bkt = bd.import_step(os.path.join(ROOT, "models/components/t_slot_option1_crossing/fx_bracket.step"))

    specs = [
        # --- Forklift Objects Compartment (R46..R55 + R44-FK) ---
        {"id": "R46", "mark": "R46-B01", "short_title": "Forklift Deck & Motor Cradle", "name": "Drop-On Sloped Deck, Stop Wall & KP08/17HS4401 Cradle",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 40%)", "notes": "Print deck-down; includes enclosed rear rack-tail safety pockets.",
         "rel_path": "models/components/objects_compartment/R46_forklift_deck_B01.stl", "shape": fork["compartment_deck"],
         "key_dims": ["Deck footprint: 180 x 175 mm", "Rear tail pockets: X=130.2..229.8", "Motor bay: NEMA17 42.3 mm + KP08"]},
        {"id": "R47", "mark": "R47-B01", "short_title": "Base Rail & Rack Left", "name": "Fixed Base Guide Rail + Elevated MOD=1.0 Rack (L)",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 40%)", "notes": "Print flat on base rail; sits strictly below 190.3 mm fork top.",
         "rel_path": "models/components/objects_compartment/R47_rail_left_B01.stl", "shape": fork["fork_base_rails"].solids()[0],
         "key_dims": ["Track length: 166.6 mm (-237.2..-70.6)", "Rack MOD=1.0, Z=183.5..187.2 mm", "Bridge clearance Z=180.7..183.1 mm"]},
        {"id": "R48", "mark": "R48-B01", "short_title": "Base Rail & Rack Right", "name": "Fixed Base Guide Rail + Elevated MOD=1.0 Rack (R)",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 40%)", "notes": "Mirrored right base rail & rack (top Z=187.2 mm).",
         "rel_path": "models/components/objects_compartment/R48_rail_right_B01.stl", "shape": fork["fork_base_rails"].solids()[1],
         "key_dims": ["Track length: 166.6 mm", "Rack MOD=1.0, Z=183.5..187.2 mm", "Bridge clearance Z=180.7..183.1 mm"]},
        {"id": "R49", "mark": "R49", "short_title": "Front 688ZZ Bearing Arch", "name": "Front 688ZZ Screw Bearing Saddle",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 40%)", "notes": "Press-fit 688ZZ (8x16x5); mounts to R44 via 2x M3.",
         "rel_path": "models/components/objects_compartment/R49_front_688zz_arch_B01.stl", "shape": fork["fork_front_arch"],
         "key_dims": ["Bearing bore: Ø16.10 mm (688ZZ)", "Mount holes: 2x Ø3.3 on 25.2 mm", "Crown <= 189.60 mm"]},
        {"id": "R50", "mark": "R50-B01", "short_title": "Stage-1 H-Carriage", "name": "Stage-1 Bridged H-Carriage (+120 mm Stroke)",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 40%)", "notes": "Slides +120 mm on T8 screw; carries R54/R55 pinions.",
         "rel_path": "models/components/objects_compartment/R50_stage1_carriage_B01.stl", "shape": fork["fork_mid_stage"],
         "key_dims": ["Sleds length: 162.5 mm", "T8 nut bore: Ø10.6 mm + 2x M3", "Pinion bosses: Ø10.4 at X=148.5/211.5"]},
        {"id": "R51", "mark": "R51-B01", "short_title": "Stage-2 Fork Tine Left", "name": "Tapered Fork Tine + Inboard Rack Tail (Left)",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 100%)", "notes": "Raised 6.8 mm blade (Z=190.30) sits strictly above R54/R55/R49.",
         "rel_path": "models/components/objects_compartment/R51_fork_tine_left_B01.stl", "shape": fork["fork_tines"].solids()[0],
         "key_dims": ["Total length: 272.0 mm (-335..-63)", "Blade top Z=190.30 (rack Z=183.5..187.2)", "Stroke: +240 mm (44 mm min lap)"]},
        {"id": "R52", "mark": "R52-B01", "short_title": "Stage-2 Fork Tine Right", "name": "Tapered Fork Tine + Inboard Rack Tail (Right)",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 100%)", "notes": "Mirrored right fork tine + rack tail (blade top Z=190.30).",
         "rel_path": "models/components/objects_compartment/R52_fork_tine_right_B01.stl", "shape": fork["fork_tines"].solids()[1],
         "key_dims": ["Total length: 272.0 mm", "Blade top Z=190.30 (rack Z=183.5..187.2)", "Stroke: +240 mm (44 mm min lap)"]},
        {"id": "R53", "mark": "R53-B01", "short_title": "Rear Under-Tray Tie Plate", "name": "Removable Under-Tray Cantilever Tie Gusset",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 40%)", "notes": "Unbolts downward (-Z) so R46 separates cleanly from R44.",
         "rel_path": "models/components/objects_compartment/R53_under_tie_B01.stl", "shape": fork["fork_under_tie"],
         "key_dims": ["Footprint: 66.0 x 70.0 mm", "Bolts: 2x M5 (KP08) + 4x M3 tie", "Flush bottom Z=155.70 mm"]},
        {"id": "R54", "mark": "R54", "short_title": "16T Spur Pinion Left", "name": "Horizontal 1:2 Spur Pinion (16T, MOD=1.0)",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG / Nylon (0.15mm, 100%)", "notes": "Press-fit F683ZZ; recessed below R51/R52 blades (max Z<=188.35).",
         "rel_path": "models/components/objects_compartment/R54_pinion_left_B01.stl", "shape": fork["fork_pinion_left"],
         "key_dims": ["Pitch Ø16.00 mm (OD 17.90 mm)", "Thickness: 3.70 mm, Z=183.5..187.2", "Hub bore: Ø7.10 mm + Ø8.20 flange"]},
        {"id": "R55", "mark": "R55", "short_title": "16T Spur Pinion Right", "name": "Horizontal 1:2 Spur Pinion (16T, MOD=1.0)",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG / Nylon (0.15mm, 100%)", "notes": "Press-fit F683ZZ; identical profile to R54 (recessed below blades).",
         "rel_path": "models/components/objects_compartment/R55_pinion_right_B01.stl", "shape": fork["fork_pinion_right"],
         "key_dims": ["Pitch Ø16.00 mm (OD 17.90 mm)", "Thickness: 3.70 mm, Z=183.5..187.2", "Hub bore: Ø7.10 mm + Ø8.20 flange"]},

        # --- Robot Body & Load Base (R40..R45, P07) ---
        {"id": "R44-FK", "mark": "R44 (Forklift)", "short_title": "Forklift Tray Base", "name": "Printed Tray Base (With T8 Trench & R46 Bores)",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 40%)", "notes": "Slice THIS STL for Forklift build.",
         "rel_path": "sim/build/meshes_objects_compartment/tray_deck.stl", "shape": fork["tray_deck"],
         "key_dims": ["Plate: 180.0 x 175.0 x 10.0 mm", "T8 trench cut down to Z=175.2", "12x M3 perimeter side holes"]},
        {"id": "R40", "mark": "R40", "short_title": "Column Collar", "name": "Body 40x25 Column Mounting Collar",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 40%)", "notes": "11 mm wrench access holes for pod column bolts.",
         "rel_path": "models/components/option1_body/opt1_body_column_collar_R40.stl", "shape": collar,
         "key_dims": ["Fits 40 x 25 x 2 mm column", "Access holes: Ø11.0 mm", "Bolts R41 boom & R42 pod"]},
        {"id": "R41", "mark": "R41", "short_title": "Tray Boom", "name": "Fixed Cantilever Boom (Collar -> TAL220)",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 45%)", "notes": "Carries load cell dead-end; keep wall infill dense.",
         "rel_path": "models/components/option1_body/opt1_body_tray_boom_R41.stl", "shape": boom,
         "key_dims": ["Span: X=26.65..228.0 mm", "TAL220 mount: M5/M4 holes", "Clear of R46 left skirt"]},
        {"id": "R42", "mark": "R42", "short_title": "Battery Counterweight Box", "name": "Electronics & Battery Counterweight Pod",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 3 walls, 25%)", "notes": "Opposite column from tray for COG balance.",
         "rel_path": "models/components/option1_body/opt1_body_battery_pod_R42.stl", "shape": bat_box,
         "key_dims": ["Mounts behind collar (-X)", "Lid screws: M3 bosses", "Houses battery + drivers"]},
        {"id": "R43", "mark": "R43", "short_title": "Battery Box Lid", "name": "Top Lid for R42 Electronics Pod",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 3 walls, 25%)", "notes": "Flat top print, no supports.",
         "rel_path": "models/components/option1_body/opt1_body_battery_lid_R43.stl", "shape": bat_lid,
         "key_dims": ["Matches R42 rim", "M3 countersunk holes"]},
        {"id": "R45", "mark": "R45", "short_title": "Live Wishbone", "name": "Live Load-Cell Wishbone (TAL220 -> R44)",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 50%)", "notes": "Transfers full cargo + fork weight via TAL220.",
         "rel_path": "models/components/option1_body/opt1_body_live_wishbone_R45.stl", "shape": live_wish,
         "key_dims": ["Bolts TAL220 live end", "Supports R44 underside pads"]},
        {"id": "P07", "mark": "P07-B01", "short_title": "Column End Plug", "name": "40x25 Column Anti-Crush End Plug",
         "process": "3D PRINT (FDM)", "qty": 2, "material": "PETG (0.2mm, 4 walls, 50%)", "notes": "Internal brace inside 40x25x2 alu tube.",
         "rel_path": "models/components/option1_dual_legs/opt1_column_anticrush_plug.stl", "shape": col_end_plugs.solids()[0],
         "key_dims": ["Fits 36 x 21 mm tube ID", "Stops M4 bolt crushing"]},
        {"id": "P07B", "mark": "P07B", "short_title": "Collar Column Plug", "name": "Mid-Column Collar Anti-Crush Insert",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 4 walls, 50%)", "notes": "Sits inside column under R40 clamp bolts.",
         "rel_path": "models/components/option1_body/opt1_body_anticrush_plug_P07B.stl", "shape": col_plug,
         "key_dims": ["Fits 36 x 21 mm tube ID", "M4 through-holes"]},

        # --- Swivel Pod x2 (P01..P08) ---
        {"id": "P01", "mark": "P01-B01", "short_title": "Swivel Carriage Pod", "name": "Swivel Carriage Pod + 40T Ring Gear",
         "process": "3D PRINT (FDM)", "qty": 2, "material": "PETG (0.2mm, 4 walls, 40% + tree supp)", "notes": "Print deck-down; tree supports under arm hub & can clamp.",
         "rel_path": "models/components/t_slot_option1/opt1_carriage_pod_40t.stl", "shape": o1["carriage_pod"],
         "key_dims": ["Swivel ring: 40T MOD=1.5", "608ZZ bore: Ø22.1 mm", "4632-370 gearbox seat"]},
        {"id": "P02", "mark": "P02-B01", "short_title": "Motor Can Cap", "name": "Removable Motor Cap (+Y Pull-Off)",
         "process": "3D PRINT (FDM)", "qty": 2, "material": "PETG (0.2mm, 4 walls, 40%)", "notes": "Clamps 4632-370 motor tail; 4x M3 bolts.",
         "rel_path": "models/components/t_slot_option1/opt1_motor_cap.stl", "shape": o1["motor_cap"],
         "key_dims": ["Can sleeve ID: ~24.8 mm", "4x M3 x 12 bolt bosses"]},
        {"id": "P03", "mark": "P03-B01", "short_title": "Pinion Cover", "name": "Outboard 623ZZ Pinion Cover",
         "process": "3D PRINT (FDM)", "qty": 2, "material": "PETG (0.2mm, 4 walls, 40%)", "notes": "Houses 623ZZ outboard bearing; 3x M3 screws.",
         "rel_path": "models/components/t_slot_option1/opt1_pinion_cover.stl", "shape": o1["pinion_cover"],
         "key_dims": ["623ZZ pocket: Ø10.1 x 4 mm", "Pulls off -Y for service"]},
        {"id": "P04", "mark": "P04-B01", "short_title": "Body Mount Servo Cage", "name": "Fixed Body Mount (TD-8125MG + Top M8 Boss)",
         "process": "3D PRINT (FDM)", "qty": 2, "material": "PETG (0.2mm, 4 walls, 40%)", "notes": "Bolts to MT0134 gantry plate & column.",
         "rel_path": "models/components/t_slot_option1/opt1_body_mount.stl", "shape": o1["body_mount"],
         "key_dims": ["Servo pocket: 40.5 x 20.8 mm", "Top axle hole: Ø8.1 / M8 nut"]},
        {"id": "P05", "mark": "P05", "short_title": "20T Swivel Servo Gear", "name": "20T Spur Gear on TD-8125MG Horn",
         "process": "3D PRINT (FDM)", "qty": 2, "material": "PETG / Nylon (0.15mm, 100%)", "notes": "Bolts to H03 alu 25T horn (4x M3 on 14 mm PCD).",
         "rel_path": "models/components/t_slot_option1/opt1_20t_servo_gear.stl", "shape": o1["gear_20t"],
         "key_dims": ["20T involute gear (2:1 w/ 40T)", "4x M3 on 14 mm PCD"]},
        {"id": "P06", "mark": "P06", "short_title": "14T Climb Pinion", "name": "14T Climbing Pinion (4632-370 6 mm D-Shaft)",
         "process": "3D PRINT (FDM)", "qty": 2, "material": "PETG / Nylon (0.15mm, 100%)", "notes": "Fits 6 mm D-shaft (5.5 mm flat) + M3 axial screw.",
         "rel_path": "models/components/t_slot_option1/opt1_pinion_14t.stl", "shape": o1["pinion"],
         "key_dims": ["Pitch Ø21.0 mm (14T MOD=1.5)", "Bore: 6 mm D-shaft (5.5 flat)"]},
        {"id": "RET", "mark": "Unmarked", "short_title": "Pinion Shaft Retainer", "name": "Pinion Axial Shaft Retainer Collar",
         "process": "3D PRINT (FDM)", "qty": 2, "material": "PETG (0.15mm, 100%)", "notes": "Too small to mark (10x10x2.5 mm); store in labelled batch bag.",
         "rel_path": "models/components/t_slot_option1/opt1_shaft_retainer.stl", "shape": o1["retainer"],
         "key_dims": ["Size: 10.0 x 2.5 x 10.0 mm", "Axial M3 x 12 bolt bore"]},
        {"id": "P08", "mark": "P08", "short_title": "Hall Sensor Bracket", "name": "49E Differential Hall Dual-Fang Bracket",
         "process": "3D PRINT (FDM)", "qty": 2, "material": "PETG (0.15mm, 4 walls, 100%)", "notes": "Holds 2x TO-92S 49E sensors 8.0 mm apart on MT0134.",
         "rel_path": "models/components/option1_dual_legs/opt1_crossing_sensor_bracket.stl", "shape": p08_bkt,
         "key_dims": ["Fang pitch: 8.0 mm c/c", "MT0134 clamp: 1x M3 x 12 + nut"]},
        {"id": "P09", "mark": "P09", "short_title": "Swivel End-Stop Bridge", "name": "A3144 Swivel End-Stop Sensor Bridge (0° / 90°)",
         "process": "3D PRINT (FDM)", "qty": 2, "material": "PETG (0.15mm, 100% infill)", "notes": "Mounts on P04 underside; holds 2x A3144 at (-11.0, +-11.0).",
         "rel_path": "models/components/option1_dual_legs/opt1_P09_swivel_sensor_bridge.stl", "shape": p09_bkt,
         "key_dims": ["Holds 2x TO-92S A3144 switches", "Detects P01 Ø3x1.8 magnet at 0°/90°"]},

        # --- Hoods & Crossings ---
        {"id": "P10-UP", "mark": "P10-UP", "short_title": "Upper Leg Hood", "name": "Upper Leg Protective Hood",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 3 walls, 25%)", "notes": "Fits upper H2 pod against rail dust.",
         "rel_path": "models/components/option1_full/opt1_P10_UP_leg_hood.stl", "shape": p10_up_raw,
         "key_dims": ["Clearance around swivel sweep"]},
        {"id": "P10-LO", "mark": "P10-LO", "short_title": "Lower Leg Hood", "name": "Lower Leg Protective Hood",
         "process": "3D PRINT (FDM)", "qty": 1, "material": "PETG (0.2mm, 3 walls, 25%)", "notes": "Shrouds lower H1 leg mechanism.",
         "rel_path": "models/components/option1_full/opt1_P10_LO_leg_hood.stl", "shape": p10_lo_raw,
         "key_dims": ["Clearance around swivel sweep"]},
        {"id": "XP-ROT", "mark": "Crossing Rotor", "short_title": "Crossing Turntable Rotor", "name": "Preloaded 608ZZ Turntable Rotor Disc",
         "process": "CNC / 3D PRINT", "qty": "1/crossing", "material": "6061-T6 Alu (CNC) or PETG 50%", "notes": "Rotates between V-rail crossing plates.",
         "rel_path": "models/components/t_slot_option1_crossing/xp_rotor.stl", "shape": xp_rotor,
         "key_dims": ["608ZZ duplex bore", "Stop pin engagement slot"]},
        {"id": "XP-RCK", "mark": "Crossing Rack", "short_title": "Crossing Turntable Rack", "name": "Turntable Rotating Rack Tooth Segment",
         "process": "CNC / 3D PRINT", "qty": "1/crossing", "material": "PETG 100% or Delrin CNC", "notes": "Meshes P06 14T climber pinion across crossings.",
         "rel_path": "models/components/t_slot_option1_crossing/xp_rack.stl", "shape": xp_rack,
         "key_dims": ["MOD=1.5 rack profile", "M5 T-nut slot mount"]},
        {"id": "FX-BKT", "mark": "Crossing Bracket", "short_title": "Fixed Crossing Bracket", "name": "Fixed 2020 Rail Crossing Corner Bracket",
         "process": "CNC / 3D PRINT", "qty": "1/crossing", "material": "6061-T6 Alu CNC / Laser or PETG", "notes": "96 x 96 x 10 mm gusseted crossing bracket; CNC alu best.",
         "rel_path": "models/components/t_slot_option1_crossing/fx_bracket.stl", "shape": fx_bkt,
         "key_dims": ["Overall: 96.0 x 96.0 x 10.0 mm", "608ZZ spigot housing mount"]},

        # --- Workshop Lathe / Shop-Cut Metal Parts ---
        {"id": "M01", "mark": "M01 (Lathe)", "short_title": "Swivel Main Axle", "name": "Common Swivel Axle (Bright Mild / Silver Steel)",
         "process": "LATHE TURNING", "qty": 2, "material": "Bright Mild / Silver Steel (NO hardened rod)", "notes": "Face square 69.0 mm; drill Ø3.3 x 12 & tap M4 x 10 both ends.",
         "rel_path": "docs/SHOPPING_LIST.md#L47", "shape": metal["M01"],
         "key_dims": ["OD: Ø8.00 h9 mm (fits 608ZZ)", "Length: 69.00 mm (faces square)", "Both ends: Ø3.3 x 12, tap M4 x 10"]},
        {"id": "M02", "mark": "M02 (Lathe)", "short_title": "Axle Spacer Bottom", "name": "Bottom Bearing Steel Axle Spacer Tube",
         "process": "LATHE TURNING", "qty": 2, "material": "Steel Tube 12 OD x 8.2 ID", "notes": "Part off & face ends flat/square to 12.0 mm.",
         "rel_path": "docs/SHOPPING_LIST.md#L48", "shape": metal["M02"],
         "key_dims": ["OD Ø12.0 mm / ID Ø8.2 mm", "Length: 12.00 mm (square faces)"]},
        {"id": "M03", "mark": "M03 (Lathe)", "short_title": "Axle Spacer Top", "name": "Top Bearing Trim-to-Snug Steel Spacer",
         "process": "LATHE TURNING", "qty": 2, "material": "Steel Tube 12 OD x 8.2 ID", "notes": "Cut 4.5 mm; lap/trim on assembly for snug 608ZZ preload.",
         "rel_path": "docs/SHOPPING_LIST.md#L49", "shape": metal["M03"],
         "key_dims": ["OD Ø12.0 mm / ID Ø8.2 mm", "Length: 4.50 mm (trim to fit)"]},
        {"id": "M04", "mark": "M04 (Lathe)", "short_title": "Axle Boss Spacer", "name": "Axle Boss Steel Washer Spacer",
         "process": "LATHE TURNING", "qty": 2, "material": "Steel Tube 12 OD x 8.2 ID", "notes": "Thin 1.5 mm faced steel ring spacer.",
         "rel_path": "docs/SHOPPING_LIST.md#L50", "shape": metal["M04"],
         "key_dims": ["OD Ø12.0 mm / ID Ø8.2 mm", "Thickness: 1.50 mm"]},
        {"id": "M11", "mark": "M11 (Lathe)", "short_title": "Crossing Pivot Housing", "name": "608ZZ Duplex Crossing Pivot Bearing Housing",
         "process": "LATHE TURNING", "qty": "1/crossing", "material": "6061-T6 Alu Bar Ø45 mm", "notes": "Spigot Ø27 g6 x 9.5; twin Ø22 H7 x 7 bearing seats; 4x Ø4.5 on 36 PCD.",
         "rel_path": "models/components/t_slot_option1_crossing/fx_pivot_housing.stl", "shape": metal["M11"],
         "key_dims": ["Spigot: Ø27.0 g6 x 9.5 mm", "Bearing bores: Ø22.0 H7 x 7 mm", "4x Ø4.5 holes on 36 mm PCD"]},
        {"id": "M13", "mark": "M13 (Lathe)", "short_title": "Crossing Front Spacer", "name": "Crossing Axle Front Bearing Steel Spacer",
         "process": "LATHE TURNING", "qty": "1/crossing", "material": "Steel Tube 12 OD x 8.2 ID", "notes": "Faced square 2.50 mm steel tube spacer in M11 pivot stack.",
         "rel_path": "docs/SHOPPING_LIST.md#L102", "shape": metal["M13"],
         "key_dims": ["OD Ø12.0 mm / ID Ø8.2 mm", "Length: 2.50 mm (square faces)"]},
        {"id": "M14", "mark": "M14 (Lathe)", "short_title": "Crossing Preload Spacer", "name": "Crossing Inner-Race Preload Steel Spacer (7.95 mm)",
         "process": "LATHE TURNING", "qty": "1/crossing", "material": "Steel Tube 12 OD x 8.2 ID (Faces Ground)", "notes": "Ground 0.05 mm shorter than M11 housing shoulder (sets 608ZZ preload).",
         "rel_path": "docs/SHOPPING_LIST.md#L103", "shape": metal["M14"],
         "key_dims": ["OD Ø12.0 mm / ID Ø8.2 mm", "Length: 7.95 mm (shoulder - 0.05 mm)"]},
        {"id": "M15", "mark": "M15 (Lathe)", "short_title": "Crossing Preload Collar", "name": "Crossing Rear Preload Steel Collar",
         "process": "LATHE TURNING", "qty": "1/crossing", "material": "Steel Tube 12 OD x 8.2 ID", "notes": "Faced square 2.00 mm rear collar under DIN 6796 disc spring.",
         "rel_path": "docs/SHOPPING_LIST.md#L104", "shape": metal["M15"],
         "key_dims": ["OD Ø12.0 mm / ID Ø8.2 mm", "Length: 2.00 mm (square faces)"]},
        {"id": "M16", "mark": "M16 (Shop Cut)", "short_title": "Stop-Pin Sleeve", "name": "Crossing M5 Stop-Pin Steel Sleeve",
         "process": "SHOP CUT / DRILL", "qty": "1/crossing", "material": "Steel Tube 10 OD x 5.2 ID", "notes": "Cut square 20.0 mm; runs on M5 x 25 stop pin inside FX-BKT slot.",
         "rel_path": "docs/SHOPPING_LIST.md#L105", "shape": metal["M16"],
         "key_dims": ["OD Ø10.0 mm / ID Ø5.2 mm", "Length: 20.00 mm"]},
        {"id": "M-T8", "mark": "CNC4759-A Cut", "short_title": "Cut T8 Lead Screw", "name": "CNC4759-A 304SS T8 Lead Screw (Trimmed)",
         "process": "SHOP CUT / DRILL", "qty": 1, "material": "304 Stainless (from 500 mm kit)", "notes": "Cut 500 mm screw -> 202.0 mm; chamfer cut end for 688ZZ.",
         "rel_path": "docs/OBJECTS_COMPARTMENT_FORKLIFT_BOM.md#L18", "shape": metal["M-T8"],
         "key_dims": ["OD Ø8.0 mm · Pitch 2 · Lead 8", "Cut length: ~202.0 mm (298 spare)", "Fits rear KP08 + coupler + front 688ZZ"]},
        {"id": "M-ROD", "mark": "MT0092 Cut", "short_title": "Hardened Smooth Shafts", "name": "MT0092 8mm Hardened Linear Rods (Optional Spine)",
         "process": "SHOP CUT / DRILL", "qty": 2, "material": "Case-Hardened Steel (Abrasive Cutoff ONLY)", "notes": "DO NOT try to tap/thread hardened MT0092 rod; abrasive wheel only.",
         "rel_path": "docs/OBJECTS_COMPARTMENT_FORKLIFT_BOM.md#L33", "shape": metal["M-ROD"],
         "key_dims": ["OD: Ø8.00 mm", "Cut length: 2 x 165.0 mm (170 spare)"]},
        {"id": "M-COL", "mark": "Column 276 mm", "short_title": "Robot Spine Column", "name": "40 x 25 x 2 mm Hollow Alu Column Bar",
         "process": "SHOP CUT / DRILL", "qty": 1, "material": "6063-T5 / 6061 Aluminum 40x25x2", "notes": "Cut square 276.0 mm; drill M4/M5 patterns for H1/H2 & R40.",
         "rel_path": "docs/SHOPPING_LIST.md", "shape": metal["M-COL"],
         "key_dims": ["Section: 40.0 x 25.0 mm (2 mm wall)", "Length: 276.0 mm"]},
    ]
    specs.extend(RR.get_rail_rack_specs())
    return specs


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    os.makedirs(OUT_SVG_DIR, exist_ok=True)
    os.makedirs(OUT_DXF_DIR, exist_ok=True)
    specs = build_all_specs()
    manifest = []
    for sp in specs:
        info = generate_drawing(sp)
        manifest.append(info)
        d = info["bbox_mm"]
        print(f"  [{info['process']:16s}] {info['id']:8s} -> {info['svg_rel']} ({d[0]} x {d[1]} x {d[2]} mm @ {info['scale']})")
    man_path = os.path.join(OUT_SVG_DIR, "manifest.json")
    with open(man_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
    print(f"Generated {len(manifest)} ISO engineering SVG/DXF sheets -> {OUT_SVG_DIR}")


if __name__ == "__main__":
    main()
