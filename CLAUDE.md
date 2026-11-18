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

| Family | Use | Shades in use |
|---|---|---|
| Blue | parts that turn with the swivel carriage | `#2E8EE6` P01 pod base, `#1B4A99` P02 motor cap, `#8FD0FF` P03 pinion cover (next: `#3F5FD6`, `#5B7FA6`) |
| Green | fixed robot-body parts | `#33B873` P04 body mount (next: `#1F7A4D`, `#7FD8A6`) |
| Amber / orange | gears, pinions, racks | `#F59E0B` P05 20T gear, `#EB7A24` P06 climb pinion, `#C2410C` / `#F59E0B` rail racks |
| Grey metallic | steel fasteners and hardware | `#9AA5B4` |
| Light grey | aluminium extrusion, plates and brackets | `#94A3B8`, `#7C8A9C` |
| Real colours | bought motors, servos, bearings | servo red `#C41E24`, motor grey `#B4BCC8`, bearing `#D8DEE6` |

- The same colour must be set in all three places: `part.color` in the builder, the RGB in `generator/pod_v3.py` /
  `assembly_v3.py`, and the THREE.js material in the viewer.

### 5. Register every printed part in
- The builder's `PART_IDS` and its printable export list.
- The README "Printed part IDs" table: ID, part, where the mark is, colour hex, print notes (orientation, supports).
- The viewer: a layer row whose label starts with the ID, plus its group, material and exploded-view offset.
- `generator/pod_v3.py` `PARTS` (pod parts) or the v3 assembly, and the BOM via `cad/export_v3_components.py`.

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
