"""Write docs/design/002-manufacturing-plan.md from models/v3/components/BOM_v3.csv (so the plan always matches the CAD).

    python cad/make_manufacturing_plan.py        # after: python cad/export_v3_components.py
"""
import collections, csv, math, os, re

HERE = os.path.dirname(os.path.abspath(__file__))
BOM = os.path.join(HERE, "..", "models", "v3", "components", "BOM_v3.csv")
OUT = os.path.join(HERE, "..", "docs", "design", "002-manufacturing-plan.md")
BED = (220.0, 220.0, 250.0)            # assumed printer build volume, mm (change here; the README says PETG)
RACK_PITCH = math.pi * 1.5             # module 1.5 -> 4.712 mm

MATERIAL = [
    (r"3D print|Print", "PETG (PA12 nylon if ordered as SLS)"),
    (r"Aluminium extrusion", "6063 aluminium extrusion (V-slot)"),
    (r"Aluminium flat bar", "6061 aluminium flat bar"),
    (r"Aluminium plate", "5052 / 6061 aluminium sheet or plate"),
    (r"Steel / aluminium flat bar", "steel or aluminium flat bar"),
    (r"Steel rod", "steel rod / tube (hardened or silver steel where it slides)"),
    (r"Plywood", "birch plywood"),
    (r"Timber", "pine battens"),
    (r"GT2 belt", "GT2 belt, 6 mm, glass-fibre core"),
    (r"basket", "the user's baskets"),
    (r"Buy", "off-the-shelf"),
]
PROCESS_TOL = [
    ("Home FDM print (PETG)", "+-0.2 to 0.3 mm; add 0.2 mm clearance on mating faces", "weakest between layers (about half the strength across them)"),
    ("Print fine (0.12 mm layers, 100 % infill)", "+-0.15 mm; check backlash on a test pair", "gears and racks"),
    ("Outsourced nylon print (SLS / MJF, PA12)", "+-0.3 mm (about +-0.1 with a good bureau)", "even strength in all directions; no supports"),
    ("Laser-cut or waterjet sheet / bar", "+-0.1 to 0.2 mm; holes can be cut at final size", "brackets, plates, scissor arms"),
    ("CNC machined / turned", "+-0.05 mm or better", "pins, sockets, pivots, bearing seats"),
    ("Off-the-shelf", "per the datasheet", "extrusion, motors, bearings, wheels, belts, hardware"),
]
# (regex on part name, note) - first match wins; fit / strength notes
NOTES = [
    (r"in-slot rack", "Tooth pitch 4.712 mm: cut every segment to a whole number of teeth (e.g. 42 teeth = 197.9 mm) so the pitch carries across joints; "
                      "segments sit in the V-slot, which keeps them in line (butt joint + a drop of adhesive, or a dovetail key)."),
    (r"printed transfer rack", "2.04 m: print in segments of whole teeth (as the in-slot racks) or buy a module 1.5 steel / nylon rack and cut it."),
    (r"printed column", "300 mm, longer than the bed. Best: aluminium bar 28 x 24 x 300 (drill the bolt pattern). Else split above the tray bracket, lap >= 50 mm, 2 dowels + 2 M4 bolts."),
    (r"climber pinion|turn drive pinion|transfer pinion", "Bore for the 6 mm shaft with a flat (H8); 100 % infill; try a test pair for 0.1-0.2 mm backlash; metal gear if it wears."),
    (r"lock socket", "Hole 11 mm for the 10 mm pin; funnel mouth 25 mm. Press a hardened steel bushing in (the wear part). Keep the printed block as thick as shown."),
    (r"cart lock pin", "10 mm, hardened, bullet nose; slides in a bush on the cart frame; spring-return. Chamfer the tip 30 deg."),
    (r"solenoid bolt|solenoid plunger", "9 mm hardened steel, bullet nose; the plunger is pushed by a push-type solenoid: check its force and stroke."),
    (r"cart docking bar", "Socket hole 11 mm with the funnel; fit a steel bushing. Bolt the ends to the cart frame posts."),
    (r"scissor arm", "Drill both arms of a pair together: pivot holes 6 mm H9. Deburr; they carry bending at the pivots."),
    (r"pivot lug", "Thin (4-6 mm) and loaded by the scissor: check stress first, or make them from 4 mm aluminium."),
    (r"flip finger", "4 mm thick, pushes a 2 kg basket: check stress first; consider 6 mm or nylon (SLS). Print on its side so layers run along the finger."),
    (r"hub plate", "6 mm; axle holes +-0.05 mm for the V-wheel eccentrics; the rail alignment depends on it."),
    (r"V-slot|extrusion", "Cut square (+-0.2 mm) and deburr; order stock lengths (2 m for the beam and floor rail)."),
    (r"rail end arm", "Cantilevered ~120 mm from the rack end post: bolt through the post web with two bolts."),
    (r"transfer beam post", "Levelling foot + base plate; bolt to the floor; set the beam height to the rails once."),
    (r"transfer beam arm", "Check the arm-to-post bolts: the whole cart rides on them."),
    (r"lead screw", "TR8x2; support the free end in the printed bearing block; grease."),
    (r"raised floor", "Level to the jack's lowered platform (38 mm top); screw the plate to the battens."),
    (r"basket", "Stand-in; use your real baskets (the model assumes 260 x 180 x 110 mm)."),
]
COLUMN_FLAG = "printed column"


def note_for(name):
    for pat, t in NOTES:
        if re.search(pat, name):
            return t
    return ""


def material_for(proc):
    for pat, m in MATERIAL:
        if re.search(pat, proc):
            return m
    return ""


def dims(size):
    try:
        return [float(x) for x in size.split(" x ")]
    except ValueError:
        return None


def fits(d):
    d = sorted(d, reverse=True)
    return d[0] <= max(BED) and d[1] <= sorted(BED, reverse=True)[1] and d[2] <= sorted(BED, reverse=True)[2]


def main():
    rows = list(csv.reader(open(BOM)))[1:]
    printed = [r for r in rows if re.search(r"print|Print", r[6]) and dims(r[4])]
    too_big = [r for r in printed if not fits(dims(r[4]))]
    by_proc = collections.Counter()
    pieces = collections.Counter()
    for r in rows:
        by_proc[r[6]] += 1
        try:
            pieces[r[6]] += int(r[3])
        except ValueError:
            pass
    cats = collections.OrderedDict()
    for r in rows:
        cats.setdefault(r[1], []).append(r)

    L = []
    w = L.append
    w("# 002 - Manufacturing plan (v3)\n")
    w("Generated by `cad/make_manufacturing_plan.py` from `models/v3/components/BOM_v3.csv`, so it always matches the CAD. "
      "Everything here is a **suggestion** from the geometry: nothing has been priced, sourced or test-printed, and tolerances and strengths "
      "are typical values, not measurements. Part IDs are the ones in the BOM and the file names.\n")
    w("## 1. How to decide: print, cut, machine or buy\n")
    w("Print what is cheap to get wrong (covers, brackets, belt clamps). Buy or cut anything structural, long or precise. "
      "Print a first article, measure it, and only then order a batch.\n")
    w("| Process | Typical accuracy | Notes |\n|---|---|---|")
    for p, t, n in PROCESS_TOL:
        w(f"| {p} | {t} | {n} |")
    w("")
    w(f"## 2. Printer and part size\n")
    w(f"Assumed printer: **{BED[0]:.0f} x {BED[1]:.0f} x {BED[2]:.0f} mm, PETG** (change `BED` in the script). "
      f"Of the {len(printed)} part types that are printed (or print-or-buy), **{len(too_big)} do not fit**:\n")
    w("| ID | part | qty | size mm | what to do |\n|---|---|---|---|---|")
    for r in too_big:
        w(f"| {r[0]} | {r[2]} | {r[3]} | {r[4]} | {note_for(r[2])} |")
    w("")
    w("Everything else printed is under 120 mm. The racks are the surprise: 36 in-slot rack pieces (318, 481 and 690 mm) and the 2 m "
      "transfer rack cannot be printed whole on a normal bed. They are segments of whole teeth in the V-slot, or bought racks cut to length.\n")
    w("## 3. Joints between printed pieces (modular parts)\n")
    w("**Hybrid joints.** Use plastic shapes to align and to take shear, and steel hardware to clamp. Never rely on a plastic-only joint in a load path.\n")
    w("| Joint | Use for | Do not use for |\n|---|---|---|\n"
      "| Dovetail or stepped lap in the plastic | aligning, taking shear and twist | pulling apart (it only locks one way) |\n"
      "| Steel 4 mm dowel pins | taking shear off the plastic, exact alignment | clamping |\n"
      "| M3 / M4 bolt + heat-set brass insert or captured nut | clamping, repeated assembly | printed threads (they strip) |\n"
      "| Plastic snap-fits / LEGO studs | covers, light alignment | anything that carries load (they creep and shear) |\n")
    w("Rules: cut where the bending moment is low; lap length >= 3x the thickness; two bolts + two dowels per joint; orient the layers so the bolts "
      "squeeze them (not peel them); PETG creeps under steady load, so keep stress low. Print a **coupon** of every new joint and pull-test it "
      "(a luggage scale is enough): require >= 4x the working load.\n")
    w("## 4. Check the strength first (shortlist)\n")
    w("Hand-calculate these, then run FEA on the worst three or four (layer direction matters: use about half the stated strength across layers):\n"
      "1. thin printed parts: flip fingers (4 mm), jack pivot lugs (4 mm), solenoid brackets (4 mm), lift carriage;\n"
      "2. the column at the tray bracket (bolt pull-out), if it is printed or split;\n"
      "3. the transfer beam, its arms and posts (the cart and robot hang from them) and the rail-end arms (cantilever);\n"
      "4. the lock pin / socket / bushing and the docking bolt: wear and a one-off shock when the cart arrives;\n"
      "5. the scissor arms and pivots (about 350 N at the lowest angle), the lead nut block, and the gears (tooth bending).\n")
    w("## 5. Print settings\n")
    w("| Class | Settings |\n|---|---|\n"
      "| General parts | PETG, 0.2 mm layers, 4 walls, 40 % gyroid (as in the v2 release notes) |\n"
      "| Gears, pinions, racks | 0.12 mm layers, 100 % infill, 4 walls; dry filament; or order PA12 nylon |\n"
      "| Thin loaded parts (fingers, lugs, brackets) | 100 % infill, layers along the main bending direction, or nylon / aluminium |\n"
      "| Mating holes | model 0.2 mm larger; ream bearing / pin holes to size |\n")
    w("## 6. Summary by process\n")
    w("| Process / source | part types | pieces |\n|---|---|---|")
    for p, n in by_proc.most_common():
        w(f"| {p} | {n} | {pieces[p] or '-'} |")
    w("")
    w("## 7. Part by part\n")
    for cat, rs in cats.items():
        w(f"### {cat}\n")
        w("| ID | part | qty | size mm | process / source | material | notes |\n|---|---|---|---|---|---|---|")
        for r in rs:
            d = dims(r[4])
            flag = ""
            if d and re.search(r"print|Print", r[6]) and not fits(d):
                flag = "**too big to print whole.** "
            w(f"| {r[0]} | {r[2]} | {r[3]} | {r[4]} | {r[6]} | {material_for(r[6])} | {flag}{note_for(r[2])} |")
        w("")
    w("## 8. Order of work\n")
    w("1. Build a **one-bay test rig** first (one rack bay, one vertical rail, the hubs, the tray and picker): it proves the printed parts, the fits and the picker before you buy three rows.\n"
      "2. Print one of each small part; measure with calipers; test-fit the mating parts; only then print batches.\n"
      "3. Order the long aluminium extrusion, the plates and the racks (or the nylon prints) in one go; they have the longest lead time.\n"
      "4. Print and pull-test the joint coupons; run the stress shortlist (section 4) before the cart, beam and docks.\n"
      "5. Replace any part that fails or wears with the next process up (print -> nylon -> machined) and note it here.\n")
    with open(OUT, "w") as f:
        f.write("\n".join(L) + "\n")
    print(f"wrote {OUT}: {len(rows)} lines, {len(too_big)} part types too big for the bed")


if __name__ == "__main__":
    main()
