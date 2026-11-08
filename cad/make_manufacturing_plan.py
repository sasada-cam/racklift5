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
    (r"cover: front drive", "Screws into the plywood tray's front edge: 2 x M3 x 16 chipboard screws, 2.5 mm pilot holes. Lower edge 3.6 mm above a lowered basket. "
                            "Print closed face down; the two ears may need a little support."),
    (r"cover: turn-drive cap", "Sits on the two mast bosses: M3 x 12 screw from the top into an M3 nut in the boss's side slot. Notch in the left wall for the mast arm."),
    (r"cover: jack motor", "Stands on the floor; its flange meets the motor bracket: 4 x M3 x 12 screws + nuts (hex pockets on the bracket's back face). One per dock."),
    (r"turn-drive mast", "Two bosses with side slots for M3 nuts (the cap's screws): print the slots open to the side and drop the nuts in before fitting the cap."),
    (r"jack: motor bracket", "4 hex nut pockets (5.8 mm across flats, 2.6 mm deep) on the back face for the motor cover's screws; plate 66 x 56 mm."),
    (r"in-slot rack", "Printed in segments of 40 whole teeth (188.5 mm; shorter end pieces at the rail ends): the cut runs through a tooth space, so "
                      "the 4.712 mm pitch carries across the joint. The V-slot keeps the pieces in line: butt joint + a drop of adhesive. One comb for the whole rail line: "
                      "every piece starts 1.856 mm before its first tooth centre and ends 1.856 mm after its last, with a 1.0 mm gap at each crossing piece, so the "
                      "pitch is 4.712 mm across every crossing (bay pitch = 93 pitches, hub spacing = 55, cart = 128). Slide the pieces in from the rail's free end, in order, "
                      "press fit + one drop of cyanoacrylate each; cap the free ends (the top of each vertical rail, the cart end) with a printed end stop."),
    (r"transfer rack segment", "40 whole teeth (188.5 mm); 10 full pieces + 1 shorter, butted on the 2 m beam (screw each piece to the beam). Or buy a module 1.5 rack."),
    (r"column bar", "Aluminium bar 28 x 24 x 300 (was a printed column that did not fit the bed): drill the bolt pattern; no joint needed."),
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
    w(f"Assumed printer: **{BED[0]:.0f} x {BED[1]:.0f} x {BED[2]:.0f} mm, PETG** (change `PRINT_BED` in `params_v3.py`; the export fails if a printed part is bigger). "
      f"Of the {len(printed)} part types that are printed (or print-or-buy), **{len(too_big)} do not fit**.\n")
    if too_big:
        w("| ID | part | qty | size mm | what to do |\n|---|---|---|---|---|")
        for r in too_big:
            w(f"| {r[0]} | {r[2]} | {r[3]} | {r[4]} | {note_for(r[2])} |")
        w("")
    w("**What the CAD and the sim now do for printing** (done in `generator/assembly_v3.py`, so the MuJoCo model has the same parts):\n"
      "- **Racks in segments:** every printed rack is cut into pieces of 40 whole teeth (188.5 mm) so the pitch carries across the joints; the earlier "
      "690 / 481 / 318 mm in-slot racks and the 2 m transfer rack could not be printed whole.\n"
      "- **Tooth phase made exact:** the crossing piece is symmetric (a tooth space at its centre), the gaps are 1.0 mm and the distances are whole pitches, so the "
      "comb is continuous through every crossing (it was 2 mm = 42 % of a pitch out for the horizontal rails).\n"
      "- **Rails supported:** a laser-cut aluminium bracket per crossing (bearing plate, stems to the posts on both sides, a support for each rail end next to the crossing piece), "
      "I-shaped supports where the last stretch passes a post, and a rail bridge half-way up the vertical rail between the two crossings.\n"
      "- **Column:** now an aluminium bar 28 x 24 x 300 instead of a 300 mm printed part.\n"
      "- **Thin loaded parts thickened:** solenoid brackets 4 -> 6 mm, jack top lugs 4 -> 6 mm and bottom lugs 6 -> 8 mm, jack motor bracket 4 -> 6 mm, lift carriage 2 -> 4 mm.\n"
      "- **Bores:** the stage-2 pulleys have a 4 mm axle bore (and bigger index holes); the flip-finger collar is 19.2 mm with a square hole for the tube; "
      "the finger shaft is 2 mm higher to make room.\n"
      "- **Export:** each printed part is laid flat on its largest face (good layer direction) and checked against the bed.\n"
      "- **Still not done:** the screw and bolt holes (to be drawn per part), the fork or slot that holds the stage-2 pulleys in the middle bar, and the flip fingers stay "
      "4 mm (the gap behind the basket is only 8 mm): print them in nylon or on their side and check them first.\n")
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
    w("## 5. Covers and fasteners\n")
    w("Printed covers protect the exposed drives and can be switched on / see-through / off in the viewer and the task page (separately for the robot "
      "and the docks). Every cover is held by **M3 screws, never by printed threads or snap-fits** (they wear out): the screw goes through a 3.4 mm "
      "clearance hole in the cover into an M3 nut in a printed slot or pocket, or (in plywood) a chipboard screw into a 2.5 mm pilot hole.\n")
    w("| Cover | Covers | Fixed to | Hardware |\n|---|---|---|---|\n"
      "| front drive | servo 1, step-up belt, cross-shaft end | tray plywood front edge | 2 x M3 x 16 chipboard screws |\n"
      "| turn-drive cap | pinion motor, lift servo, carriage | two bosses on the mast | 2 x M3 x 12 + 2 nuts + 2 washers |\n"
      "| jack motor (x3) | NEMA17 + gearhead | enlarged motor bracket | 4 x M3 x 12 + 4 nuts + 4 washers each |\n")
    w("**Not covered yet:** servo 2 and its gear pair (the corner is too tight against the finger shaft), the two lock solenoids, the climb "
      "motors and pinions in the hubs, and the scissor itself. They need their own cover design.\n")
    w("**Totals:** 14 x M3 x 12 machine screws, 2 x M3 x 16 chipboard screws, 14 M3 hex nuts, 14 M3 washers - buy about 20 % spares.\n")
    w("**Sourcing in Sri Lanka** (checked on the web on 2026-11-08; stock and prices change, and this is not a quote): M3 / M4 are standard metric sizes, "
      "which I expect any hardware or electronics shop to stock, but a search could not confirm local shops, so check before ordering. "
      "**Heat-set brass inserts** are available only as imports (Desertcart / Ubuy Sri Lanka): a 420-piece M3-M5 kit was about LKR 14,900 and a 400-piece kit "
      "about LKR 31,900; a 100-piece M3 pack (about LKR 940) was out of stock; the heat-press tool was about LKR 18,900. That is why the design uses plain "
      "nuts in printed pockets, and inserts are only an optional upgrade for parts you take apart often.\n"
      "Sources: [M3 insert 100-pack](https://www.desertcart.lk/products/812137818-100-pack-m3-threaded-inserts-heat-set-inserts-m3x4x5-female.md), "
      "[M3-M5 insert kit](https://www.desertcart.lk/products/600782778-420pcs-m3-m4-m5-threaded-inserts-assortment-kit-for-3d.md), "
      "[heat-press insert tool](https://www.ubuy.com.lk/en/product/7VBIO3LMU-3dzwman-vertical-heat-pressing-machine-heat-set-insert-tool-brass-inserts-3d-printing-heat-set-insert-tip-for-thread-knurled-nut).\n")
    w("## 6. Print settings\n")
    w("| Class | Settings |\n|---|---|\n"
      "| General parts | PETG, 0.2 mm layers, 4 walls, 40 % gyroid (as in the v2 release notes) |\n"
      "| Gears, pinions, racks | 0.12 mm layers, 100 % infill, 4 walls; dry filament; or order PA12 nylon |\n"
      "| Thin loaded parts (fingers, lugs, brackets) | 100 % infill, layers along the main bending direction, or nylon / aluminium |\n"
      "| Mating holes | model 0.2 mm larger; ream bearing / pin holes to size |\n")
    w("## 7. Summary by process\n")
    w("| Process / source | part types | pieces |\n|---|---|---|")
    for p, n in by_proc.most_common():
        w(f"| {p} | {n} | {pieces[p] or '-'} |")
    w("")
    w("## 8. Part by part\n")
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
    w("## 9. Order of work\n")
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
