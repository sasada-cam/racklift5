# RackLift5 v3 - shopping list: swivel pods + rail crossings

Every part that is bought, cut or machined gets a number, the same way printed parts carry P-numbers.

| Prefix | Meaning |
|---|---|
| `E` | motors and servos |
| `H` | hardware bought as-is (bearings, screws, nuts, washers, T-nuts, kits) |
| `M` | workshop parts cut or machined from stock (lathe / cutting shop) |
| `P` | 3D-printed parts (engraved ID + batch, see `CLAUDE.md`) |

The quantities below are for the whole build:
- **Swivel pods:** 2 (one per hub). The robot has 2 hubs, and each has one pod.
- **Rail crossings:** 24 (12 bays × 2 rails). Each crossing has one turning rail piece on its own pivot.

All screws are metric ISO/DIN. Plain zinc-plated 8.8 steel is fine, or A2 stainless if you want no rust.

Where to buy:
- screws, nuts and washers: Pettah fastener / hardware shops
- bearings, motors and servos: tronic.lk (or bearing shops)
- `M` parts: any lathe or metal-cutting shop

Take this page and a sample part to the shop.

---

## 1. Swivel pod (×2)

### Motors, servos, kits

| ID | Part | Material / spec | Size | Per pod | Total | Where |
|---|---|---|---|---|---|---|
| E01 | 4632-370 worm gear motor, 12 V, 100 RPM | steel gearbox, 6 mm D-shaft (5.5 mm flat), M3 axial hole in shaft | gearbox 32 × 22.5 × 46, can Ø24.4 × 30.8 | 1 | 2 | tronic.lk |
| E02 | TD-8125MG servo, 180°, 25T spline | metal gear | body 40 × 20.5, ears 54.5 span, ear holes Ø4.2 on 49.5 × 10 | 1 | 2 | tronic.lk |
| E03 | 49E linear analog Hall sensor IC (TO-92S) | ratiometric linear Hall (`ic_a` + `ic_b`, `8.0 mm` apart in `P08` head; `V_A − V_B = 0` centers crossing) | TO-92S (`4.0 × 3.0 × 1.5`) | 2 | 4 | tronic.lk |
| E04 | A3144 digital Hall switch IC (TO-92S) | unipolar digital switch (`0°` & `90°` swivel end-stops on `P09` under `P04`) | TO-92S (`4.0 × 3.0 × 1.5`) | 2 | 4 | tronic.lk |
| H01 | OpenBuilds MT0134 gantry plate | aluminium | 65.5 × 65.5 × 3 | 1 | 2 | 3D-printer / CNC shop |
| H02 | Delrin V-wheel kit (wheel + bearings + 6 mm spacer + M5 screw + nyloc) | Delrin, 2 × 625ZZ inside | wheel Ø24 × 10.95, spacer 6 mm | 4 | 8 | 3D-printer / CNC shop |
| H03 | 25T aluminium servo horn, round | CNC aluminium | Ø20 × 3.5, 4 × M3 tapped on 14 mm PCD (bolt-circle diameter) | 1 | 2 | tronic.lk (often with the servo) |
| H04 | 608ZZ ball bearing (608-2RS = sealed) | steel | 8 × 22 × 7 | 2 | 4 | tronic.lk / bearing shop |
| H27 | NdFeB N35/N52 disc magnet (swivel target) | neodymium (glued in `P01` hub magnet lug at `(-11.0, +11.0)`) | Ø3 × 1.8 | 1 | 2 | tronic.lk / magnet shop |

### Swivel axle + spacers (workshop)

| ID | Part | Material | Size / work | Per pod | Total |
|---|---|---|---|---|---|
| M01 | Common swivel axle | 8 mm bright mild steel or silver steel. **Not** a hardened linear rod, because it can't be tapped. | Ø8 h9, **69 mm** long, faces square; both ends drilled Ø3.3 × 12 deep and tapped **M4 × 10** | 1 | 2 |
| M02 | Axle spacer, bottom | steel tube 12 OD × 8.2 ID (or turned from 12 mm bar) | **12.0 mm** long, faces square | 1 | 2 |
| M03 | Axle spacer, top-bearing | same tube | **4.5 mm**: trim on assembly so the top stack is snug | 1 | 2 |
| M04 | Axle spacer, boss | same tube | **1.5 mm** | 1 | 2 |

### Screws, nuts, washers

| ID | Part | Standard | Per pod | Total | Used for |
|---|---|---|---|---|---|
| H10 | M4 × 10 socket cap | DIN 912 | 2 | 4 | axle ends (bottom + top) |
| H11 | M4 large washer, 12 mm OD | DIN 9021 | 1 | 2 | under the bottom axle screw (bears on the 608 inner race only) |
| H12 | M4 washer | DIN 125 | 5 | 10 | top axle screw (1) + over EPDM rubber washers on tube front (4) |
| H13 | M4 × 35 socket cap | DIN 912 | 4 | 8 | front of `40 × 25 × 2 mm` hollow alu column + `P07` anti-crush plug → captive `H25` M4 hex nut in `P04` |
| H25 | M4 hex nut (`7.0 mm AF × 3.2`) | DIN 934 | 4 | 8 | slides into `P04` side-entry captive hex nut slots |
| H26 | M4 EPDM / neoprene rubber washer (`9 × 4.2 × 1.2`) | EPDM rubber | 4 | 8 | between `H12` steel washer and aluminium column front face (absorbs hot/cold expansion & PETG creep) |
| H14 | M5 × 12 button head | ISO 7380 | 4 | 8 | carriage pod deck → MT0134 plate |
| H15 | M5 nyloc nut | DIN 985 | 4 | 8 | under the MT0134 plate |
| H16 | M3 × 6 button head | ISO 7380 | 4 | 8 | 4632-370 gearbox → pod front wall (low head clears the pinion) |
| H17 | M3 × 12 socket cap | DIN 912 | 6 | 12 | 20T gear → aluminium horn (4) + `623ZZ` pinion outboard bearing → motor shaft tip (1) + `P08` corner pocket → `MT0134` slot (1) |
| H18 | M3 × 14 pan head thread-forming ("PT", for plastic) | - | 4 | 8 | servo ears → body mount |
| H19 | M3 × 12 pan head thread-forming ("PT") | - | 5 | 10 | motor cap → pod (4), pinion cover → arm post (1) |
| H20 | M3 × 10 pan head thread-forming ("PT") | - | 2 | 4 | pinion cover → ear beside the front wall |
| H21 | M3 × 10 socket cap | DIN 912 | 3 | 6 | motor-can clamp: cap half ↔ P01 half |
| H22 | M3 hex nut (or nyloc DIN 985) | DIN 934 | 4 | 8 | motor-can clamp (3; use M3 × 12 if substituting 4 mm nyloc) + `P08` captive bottom nut pocket (1) |
| H24 | 623ZZ miniature ball bearing (`3 × 10 × 4`) | steel | 1 | 2 | `P06` outboard straddle support seated in `P03` front boss (replaces `H23`) |
| - | M3 × 8 servo spline screw | - | 1 | 2 | supplied with the servo |

### Printed (PETG, see the README "Printed part IDs" table)

Print settings: 0.2 mm layers, 4 walls, 40 % gyroid. Gears: 0.12 mm layers, 100 % infill.

| ID | Part | Overall size mm (x y z) | Volume | Per pod | Total | Print notes |
|---|---|---|---|---|---|---|
| P01 | Carriage pod (deck, tower, 40T, front wall, arm + post, can clamp front) | 104.9 × 64.7 × 86.4 | 58.7 cm³ | 1 | 2 | deck down, tree supports (arm hub, clamp lower edge) |
| P02 | Motor cap | 49.2 × 26.5 × 86.4 | 13.2 cm³ | 1 | 2 | back face down |
| P03 | Pinion cover | 48.5 × 19.6 × 31.5 | 7.3 cm³ | 1 | 2 | front plate down |
| P04 | Body mount (axle column, full-length cheeks, solid web) | 88.8 × 38.0 × 55.0 | 77.0 cm³ | 1 | 2 | top plate down |
| P05 | 20T swivel gear | 27.1 × 27.1 × 8.0 | 3.4 cm³ | 1 | 2 | flat, 100 % |
| P06 | 14T climb pinion | 23.6 × 9.8 × 24.0 | 2.1 cm³ | 1 | 2 | flat, 100 % |
| P07 | Column anti-crush plug (inside `40 × 25 × 2 mm` hollow alu column) | 35.8 × 20.8 × 80.0 | - | 1 | 2 | flat, slides inside column top & bottom (`M4 × 35` H13 bores) |
| P08 | Crossing `49E` differential Hall bracket (`MT0134` corner pocket + gusseted drop arm, `opt1_crossing_sensor_bracket.stl`) | 27.5 × 30.0 × 36.5 | - | 1 | 2 | wraps `MT0134` `(-X,+Y)` edge; holds `2 × 49E` chips **`8.0 mm` apart** (`(-46,+42)` & `(-38,+42)`), locked via `H17` + `H22` |
| P09 | Top swivel `A3144` sensor bridge (`P04` underside) + `P01` hub magnet lug | 4.8 × 28.4 × 3.5 | - | 1 | 2 | holds `2 × A3144` at `(-11.0, ±11.0)` (`0°` / `90°`, clear of `H25` nuts & boss) + `H27` `Ø3` magnet lug on `P01` rim |
| - | Shaft retainer (unmarked) | 10.0 × 2.5 × 10.0 | 0.2 cm³ | 1 | 2 | flat |

---

## 2. Rail crossing pivot (×24)

### Workshop parts

| ID | Part | Material | Size / work | Per crossing | Total |
|---|---|---|---|---|---|
| M10 | Rotor disc | 6 mm aluminium plate (laser / waterjet) | Ø42 × 6. Centre: Ø8.2 through + **hex pocket 13 mm AF × 5.6 deep** (M8 bolt head). 2 × Ø5.5 holes at ±14 mm on a diameter, counterbored Ø10 × 3 for M5 button heads | 1 | 24 |
| M11 | Pivot housing | aluminium bar Ø45 (lathe) | spigot **Ø27 g6** × 9.5 · flange Ø45 × 5 · body Ø27 × 10 · bores **Ø22 H7** × 7 (both ends) with a Ø18 shoulder 8 mm long between · 4 × Ø4.5 on a 36 mm circle at 45° | 1 | 24 |
| M12 | Crossing bracket (existing part, new holes) | 3–10 mm aluminium plate | bore **Ø27.1 H7** at the pivot; 4 × M4 tapped on a 36 mm circle at 45° (tap drill Ø3.3) | 1 | 24 |
| M13 | Front spacer | steel tube 12 × 8.2 | **2.5 mm** | 1 | 24 |
| M14 | Inner-race spacer (sets the preload) | steel tube 12 × 8.2, faces ground | **7.95 mm** = 0.05 shorter than the housing shoulder. Measure the shoulder and make it 0.05 shorter. | 1 | 24 |
| M15 | Preload collar | steel tube 12 × 8.2 | **2.0 mm** | 1 | 24 |
| M16 | Stop-pin sleeve | steel tube 10 × 5.2 | **20 mm** | 1 | 24 |

### Bought

| ID | Part | Standard / size | Per crossing | Total | Used for |
|---|---|---|---|---|---|
| H04 | 608ZZ ball bearing | 8 × 22 × 7 | 2 | 48 | preloaded pair in M11 |
| H30 | M8 × 40 hex bolt, 8.8 | DIN 933 | 1 | 24 | pivot (head in the M10 hex pocket) |
| H31 | M8 nyloc nut | DIN 985 | 1 | 24 | preload |
| H32 | M8 washer | DIN 125 | 1 | 24 | under the nyloc |
| H33 | M8 disc spring (Belleville) | DIN 6796 | 1 | 24 | keeps the preload. May need ordering online; without it, tighten to "no play, still turns freely". |
| H34 | M5 × 10 button head | ISO 7380 | 2 | 48 | rotor disc → rail piece |
| H35 | M5 drop-in T-nut for 2020 slot (6 mm opening; sits inside the slot behind the lips, about 10 × 10 × 4.4) | - | 3 | 72 | rotor screws (2) + stop pin (1), in the turning piece's back slot |
| H36 | M4 × 12 socket cap | DIN 912 | 4 | 96 | housing → bracket |
| H37 | M5 × 25 socket cap | DIN 912 | 1 | 24 | stop pin |
| H38 | M5 washer | DIN 125 | 1 | 24 | stop pin |
| H39 | M5 × 12 socket cap + M5 washer | DIN 912 / 125 | 2 + 2 | 48 + 48 | bracket → rack post flange |
| H40 | NdFeB N35/N52 disc magnet (`Ø5 × 1.2 mm`) | epoxy / CA glue | 2 | 48 | crossing stops on `M12` bracket at `(-42, +42)` (vertical) & `(-42, -42)` (horizontal), **same pole facing out** |

---

## 3. Body, tray, picker + leg hoods (×1 robot, `cad/build_option1_body.py`, `cad/build_option1_full.py`)

### Bought

| ID | Part | Spec | Qty | Used for |
|---|---|---|---|---|
| E05 | TAL220 bar load cell, 5 kg | 80 × 12.7 × 12.7, M5 (fixed end) / M4 (live end) holes | 1 | tray weighing: boom -> load cell -> live wishbone -> tray |
| E06 | HX711 load-cell amplifier board | 24-bit ADC | 1 | reads E05 |
| H50 | M4 × 30 socket cap | DIN 912 | 4 | R40 collar -> column front (through the column + P07-BODY plug, nut inside) |
| H13 | M4 × 35 socket cap (same as the pod column bolts) | DIN 912 | +4 | R40 collar wings, lower row (z 146) -> R41 / R42 |
| H52 | M4 × 40 socket cap | DIN 912 | 4 | R40 collar wings, upper row (z 210, through the rear strap) |
| H53 | M5 × 20 socket cap | DIN 912 | 2 | R41 boom -> E05 fixed end |
| H54 | M4 × 16 socket cap | DIN 912 | 2 | R45 wishbone -> E05 live end |
| H55 | M4 × 14 socket cap | DIN 912 | 4 | R45 wishbone -> R44 tray (captive nuts in the tray) |
| H25 | M4 hex nut | DIN 934 | +16 | collar (4) + wings (8) + tray (4) |
| H26 | M4 EPDM washer | 9 × 1.2 | +4 | collar column bolts |
| H12 | M4 washer | DIN 125 | +4 | collar column bolts |
| H56 | M3 × 14 socket cap | DIN 912 | 4 | leg hoods P10-UP / P10-LO -> P04 cheeks (2 per hood; 3.3 mm hole drilled through each P04 cheek) |
| H22 | M3 hex nut | DIN 934 | +4 | hood bolts (outside the hood ear) |

### Printed (PETG)

| ID | Part | Notes |
|---|---|---|
| R40 | Body column collar | 11 mm access holes over the lower pod's column bolts (z 152) |
| R41 | Tray boom | |
| R42 | Battery / controller box | |
| R43 | Battery box lid | |
| R44 | Printed tray 180 × 175 | basket rides centred between the fingers (y -210..-80) |
| R45 | Live wishbone | |
| P07 | Column anti-crush plugs: H2 (z 82..162), BODY (z 162.5..226), H1 (z 258..338) | 3 per column, holes for every bolt through the column |
| P10-UP / P10-LO | Leg hoods (protective shrouds) | 11 mm access holes over their pod's column bolts; held by H56 |

---

## 4. Totals to buy (pods + crossings + body)

> **Objects Compartment (2-Stage Forklift):** See [`docs/OBJECTS_COMPARTMENT_FORKLIFT_BOM.md`](./OBJECTS_COMPARTMENT_FORKLIFT_BOM.md) for the exact `N20` leadscrew, `8 mm` Tronic.lk (`MT0092`) smooth shafts, `F683ZZ` / `684ZZ` bearings, `GT2` belt, printed STLs, and skipped rails/brackets list.

| Item | Qty |
|---|---|
| 608ZZ bearings (H04) | 4 + 48 = **52** |
| 623ZZ bearings (H24) | **2** |
| 8 mm bright steel rod (M01) | 2 × 69 mm (one 300 mm length is plenty) |
| 12 × 8.2 steel tube (M02–M04, M13–M15) | 2 × (12 + 4.5 + 1.5) + 24 × (2.5 + 7.95 + 2) ≈ 340 mm + cutting waste: buy **500 mm** |
| 10 × 5.2 steel tube (M16) | 24 × 20 mm: buy **600 mm** |
| M3 screws | H16 8 · H17 12 · H18 8 · H19 10 · H20 4 · H21 6 · H56 4 |
| M3 nuts (H22) | 12 (`DIN 934`) |
| Hall sensors & magnets | E03 `4 × 49E` linear Hall · E04 `4 × A3144` digital Hall · H27 `2 × Ø3 × 1.8 mm` · H40 `48 × Ø5 × 1.2 mm` |
| M4 screws | H10 4 · H13 12 (`M4×35`) · H36 96 · H50 4 · H52 4 · H54 2 · H55 4 |
| M4 nuts (H25) | 24 (`DIN 934`) |
| M4 washers | H11 2 · H12 14 · H26 12 (`EPDM rubber`) |
| M5 screws (body) | H53 2 |
| Load cell | E05 TAL220 5 kg + E06 HX711 |
| Aluminium column box bar | 1 × `40 × 25 × 2.0 mm × 276 mm` (z 74..350: the lower pod is mirrored, so the column starts above it) + 3× `P07` PETG anti-crush plugs |
| M5 screws | H14 8 · H34 48 · H37 24 · H39 48 |
| M5 nyloc (H15) | 8 |
| M5 washers | H38 24 · H39 48 |
| M5 T-nuts (H35) | 72 |
| M8 × 40 bolts (H30) / nyloc (H31) / washers (H32) / disc springs (H33) | 24 each |

Buy roughly 10 % extra of every screw, nut and washer: small screws get lost.
