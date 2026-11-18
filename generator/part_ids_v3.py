"""Printed-part IDs, batch and colour scheme for the v3 rack + robot (everything outside the swivel pod, which has its own
P-numbers in cad/build_t_slot_option1.py).  Rules: CLAUDE.md "Every 3D-printed part: ID + batch + colour".

    lookup(name)          -> (ID, hex colour) for a printed part, or None
    colour(name, default) -> RGB (0..255) to draw the part with (the registry colour for printed parts, else default)

Letters: R robot body / tray / picker, X rails / crossings / in-slot racks, C transfer cart + beam, D docks + jacks, K covers.
Families: each letter has its own colour family; gears, pulleys, pinions and racks are amber / orange whatever their letter.
Every entry gets its own shade, so parts that come apart never share one.
"""
import re

BATCH = "B01"                    # bump for every print run

# family -> shades (assigned in order of the entries below)
FAMILY = {
    "green":  ["#33B873", "#1F7A4D", "#7FD8A6", "#4CAF50", "#2E6B47", "#A5D6A7", "#66BB6A", "#388E3C", "#81C784",
               "#1B5E20", "#C8E6C9", "#43A047", "#00897B", "#26A69A", "#80CBC4"],
    "amber":  ["#F59E0B", "#EB7A24", "#D97706", "#FBBF24", "#C2410C", "#FDBA74", "#B45309", "#FCD34D", "#EA580C",
               "#FB923C", "#92400E", "#FDE68A", "#F97316", "#FFB74D", "#E65100", "#FFCC80", "#BF360C", "#FFA726",
               "#F57C00", "#FF8F00", "#FFE0B2", "#E67E22"],
    "teal":   ["#06B6D4", "#0E7490", "#67E8F9"],
    "pink":   ["#EC4899", "#9D174D", "#F9A8D4"],
    "purple": ["#8B5CF6", "#5B21B6", "#C4B5FD", "#7C3AED", "#A78BFA", "#4C1D95", "#DDD6FE"],
    "light":  ["#E5E7EB", "#CBD5E1", "#F8FAFC"],
}

# (exact base name, ID, family).  In-slot rack variants of different length are numbered at export (X10, X11, ...).
ENTRIES = [
    # R: robot body, tray, picker (fixed / moving robot parts: green)
    ("printed lift carriage", "R01", "green"),
    ("printed turn-drive mast", "R02", "green"),
    ("printed front post (rod + cross-shaft bearings)", "R03", "green"),
    ("printed arm post", "R04", "green"),
    ("printed arm post (rear, idler axle)", "R05", "green"),
    ("printed basket stop", "R06", "green"),
    ("printed flip finger (front)", "R07", "green"),
    ("printed flip finger (rear)", "R08", "green"),
    ("printed stage-1 belt clamp (middle bar)", "R09", "green"),
    ("belt anchor (inner bar)", "R10", "green"),
    ("belt anchor (outer bar)", "R11", "green"),
    ("finger shaft bearing (inner bar)", "R12", "green"),
    ("lock solenoid bracket (docking bolt)", "R13", "green"),
    ("lock solenoid bracket (release plunger)", "R14", "green"),
    # R: robot gears + pulleys (amber)
    ("GT2 25T pulley (servo 1)", "R20", "amber"),
    ("GT2 33T pulley (stage-1 drive)", "R21", "amber"),
    ("GT2 8T pulley (step-up, cross shaft)", "R22", "amber"),
    ("GT2 idler (stage-1 belt)", "R23", "amber"),
    ("GT2 pulley (right rod)", "R24", "amber"),
    ("GT2 pulley (servo 2)", "R25", "amber"),
    ("finger drive gear (left rod)", "R26", "amber"),
    ("finger drive gear (servo 2)", "R27", "amber"),
    ("stage-2 pulley (extend)", "R28", "amber"),
    ("stage-2 pulley (retract)", "R29", "amber"),
    ("turn drive pinion 16T (robot)", "R30", "amber"),
    # X: rails + crossings (teal), in-slot racks (amber)
    ("printed rail bridge", "X01", "teal"),
    ("cart lock socket (printed, funnel lead-in, on the lower rail-end arm)", "X02", "teal"),
    ("printed in-slot rack", "X10", "amber"),                 # + one ID per length variant: X10, X11, ...
    ("printed in-slot rack (turntable)", "X30", "amber"),
    ("printed in-slot rack (cart)", "X31", "amber"),
    # C: transfer cart + beam
    ("cart docking bar (printed, with the bolt socket)", "C01", "pink"),
    ("printed transfer rack segment (module 1.5)", "C10", "amber"),
    # D: docks + jacks (purple)
    ("jack: motor bracket (printed)", "D01", "purple"),
    ("jack: screw bearing block (printed)", "D02", "purple"),
    ("jack: lead nut block (printed)", "D03", "purple"),
    ("jack: slider shoe (printed)", "D04", "purple"),
    ("jack: bottom pivot lug", "D05", "purple"),
    ("jack: top pivot lug", "D06", "purple"),
    ("jack: shaft coupling + index flag", "D07", "purple"),
    # K: covers (light)
    ("cover: front drive (servo 1 + belt)", "K01", "light"),
    ("cover: jack motor", "K02", "light"),
    ("cover: turn-drive cap", "K03", "light"),
]

_SHADE = {}
_used = {f: 0 for f in FAMILY}
for _n, _i, _f in ENTRIES:
    _SHADE[_n] = FAMILY[_f][_used[_f] % len(FAMILY[_f])]
    _used[_f] += 1
_BY_NAME = {n: (i, _SHADE[n]) for n, i, f in ENTRIES}


def base_name(n):
    n = re.sub(r" \((row|hub|dock|corner|turret|face) \d+\)$", "", n)
    return re.sub(r" \((upper|lower) hub\)$", "", n)


def lookup(name):
    return _BY_NAME.get(base_name(name))


def rgb(hexcol):
    h = hexcol.lstrip("#")
    return tuple(int(h[k:k + 2], 16) for k in (0, 2, 4))


def colour(name, default):
    hit = lookup(name)
    return rgb(hit[1]) if hit else default


def mark(part_id, short=False):
    return part_id if short else f"{part_id}-{BATCH}"
