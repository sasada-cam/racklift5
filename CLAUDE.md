# RackLift5 - project rules

## Every 3D-printed part: ID + batch + colour

These rules apply to every part that is 3D printed (anything named "printed ..." or exported for printing), in every
builder (`cad/*.py`, `generator/*.py`). A new or changed printed part is not done until all of them hold.

### 1. Part ID
- Format `<assembly letter><2 digits>`, e.g. `P03`. Assembly letters:
  - `P` swivel pod (`cad/build_t_slot_option1.py`)
  - `R` robot body / tray / picker
  - `X` rails, crossings and in-slot racks
  - `D` docks and jacks
  - `C` transfer cart and beam
  - `K` covers
- Numbers are unique within the letter. A part that has never been printed may be renumbered. Once a part has been
  printed, its ID is frozen: a merged or retired part's number is never reused.
- IDs live in one dict per builder (`PART_IDS = {"part_key": "P01", ...}`), never inline strings.

### 2. Batch number
- Format `B<2 digits>` (`B01`, `B02`, ...). One `BATCH` constant per builder; bump it for each print run.
- The engraved mark is `<ID>-<BATCH>`, e.g. `P02-B01` (`mark(key)` in `build_t_slot_option1.py`).

### 3. Engraving
- Cut 0.6 mm deep (3 layers at 0.2 mm), bold font, on a flat face that stays visible after assembly. Use the
  `engrave()` helper, never a raised mark.
- Never put a mark on bearing seats, gear teeth, sliding or clamping faces, or a face that prints on supports.
- The text must fit the face with at least 1 mm margin. Use 5 mm text on large faces, 3 mm on covers, and 2 mm minimum.
- If `ID-BATCH` does not fit, engrave the ID only. If the ID does not fit either (part under about 10 mm), leave it
  unmarked and list it as unmarked in the README.
- Text on a curved face runs along the axis of the curve (vertical on a cylinder), so the depth stays even.
- Verify each mark: the part is still one valid solid, and its volume dropped (the text really cut in).

### 4. Colour scheme
One colour family per function, and a different shade for each separately printed part within a family. Parts that
come apart from each other (base vs. removable cap or cover) must never share a shade, so every split shows in the viewer.

| Family | Use | Shades |
|---|---|---|
| Blue | swivel pod parts that turn with the carriage (P) | `#2E8EE6` P01, `#1B4A99` P02, `#8FD0FF` P03 |
| Green | fixed robot-body parts (P04 body mount `#33B873`) and robot body / tray / picker parts (R01-R14) | see `part_ids_v3.FAMILY["green"]` |
| Amber / orange | gears, pulleys, pinions, racks: any letter (P05, P06, R20-R30, X10.., X30, X31, C10) | `part_ids_v3.FAMILY["amber"]` |
| Teal | rail / crossing printed parts (X01 rail bridge, X02 cart lock socket) | `part_ids_v3.FAMILY["teal"]` |
| Pink | transfer cart parts (C01) | `part_ids_v3.FAMILY["pink"]` |
| Purple | docks and jacks (D01-D07) | `part_ids_v3.FAMILY["purple"]` |
| Light grey | covers (K01-K03) | `part_ids_v3.FAMILY["light"]` |
| Grey metallic | steel fasteners and hardware | `#9AA5B4` |
| Light grey (aluminium) | aluminium extrusion, plates and brackets | `#94A3B8`, `#7C8A9C` |
| Real colours | bought motors, servos, bearings | servo red `#C41E24`, motor grey `#B4BCC8`, bearing `#D8DEE6` |

- The v3 parts (R, X, C, D, K) take their ID, family and shade from `generator/part_ids_v3.py`: add a new printed part
  there (`ENTRIES`). Shades are handed out in order, so every part gets its own. `pixi run export-components` engraves
  them automatically (`cad/engrave_auto.py`); check the "engraved mark" column of `BOM_v3.csv` for any "unmarked".
- The same colour must be set in all three places: `part.color` in the builder, the RGB in `generator/pod_v3.py` /
  `assembly_v3.py`, and the THREE.js material in the viewer.

### 5. Register every printed part in
- The builder's `PART_IDS` and its printable export list.
- The README "Printed part IDs" table: ID, part, where the mark is, colour hex, print notes (orientation, supports).
- The viewer: a layer row whose label starts with the ID, plus its group, material and exploded-view offset.
- `generator/pod_v3.py` `PARTS` (pod parts) or the v3 assembly, and the BOM via `cad/export_v3_components.py`.

## Bought and workshop parts: `docs/SHOPPING_LIST.md`
- Every part that is not printed gets a number:
  - `E`: motors and servos
  - `H`: hardware bought as-is
  - `M`: cut or machined from stock (lathe / cutting shop)
- Each entry gives: material, exact size and work (length, bores and fits, tapped holes, tolerances), quantity per assembly
  and total, and where to buy it. Keep the totals table at the bottom correct.
- Model the real fixing, never a part stuck onto a face. For example, a screw into a rail slot ends in a T-nut inside the
  slot.

## Hardware
- Metric ISO/DIN parts only (DIN 912, ISO 7380, DIN 933/934/985, DIN 125, 608ZZ), bought in Sri Lanka (Pettah
  hardware shops, tronic.lk). Flag anything that probably has to be ordered online.
- Every screw, nut and bolt is modelled in the CAD and listed in `FASTENERS` and in the README fastener table.

## Before committing a CAD change
1. Each changed part is a single valid solid.
2. For removable parts, check the assembly and removal paths. Move every solid of the moving part separately in the
   world frame (`solid.translate(...)` per solid). Never move a whole compound: it gives false overlaps.
3. `chk_pod`-style swivel check, 0 to 90 deg: no clashes beyond intended thread engagement.
4. `pixi run python cad/build_option1_crossing.py` -> 0 collisions, both stops blocked.
5. `pixi run clearance-v3` -> 157/157 poses clean.
6. Re-export meshes (`cad/build_t_slot_option1.py`) so the viewer shows the change.
