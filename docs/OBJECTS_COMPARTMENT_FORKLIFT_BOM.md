# Objects Compartment (2-Stage Forklift) — Parts & Shopping List

Exact parts, quantities, verified dimensions, and local Sri Lanka sourcing (`tronic.lk` / `duino.lk` / hardware shops) for the **2-Stage Telescopic Forklift Objects Compartment** (`R46`–`R55`, batch `B01`, beltless Horizontal `1:2` Rack-and-Pinion Stage-2), mounted on the isolated load base (`R41` + `TAL220` + `R45` + `R44`).

- **CAD Builder:** [`cad/build_objects_compartment.py`](../cad/build_objects_compartment.py)
- **3D Studio Viewer:** [`sim/view_objects_compartment.py`](../sim/view_objects_compartment.py)
- **Exported STLs / STEP:** [`models/components/objects_compartment/`](../models/components/objects_compartment/)
- **2D Blueprints (SVG + DXF) & Interactive Print/Build Browser:** [`docs/drawings/`](./drawings/) (`R44-FK`, `R46`–`R55`, `M-T8`, `M-ROD`) or run `pixi run python sim/view_dashboard.py` -> **`🖨️ Print & Build Parts (2D/3D)`** (`/print-parts/`)
- **Robot Chassis / Legs / Load Cell BOM:** [`docs/SHOPPING_LIST.md`](./SHOPPING_LIST.md)

---

## 1. Exact Shopping List — What to Buy & Quantities (1× Robot Compartment)

### A. Drive, Bearings & Belts (`tronic.lk` / Bearing Shop)

| ID | Exact Part Name | Spec / Size | Qty to Buy | Used For |
|---|---|---|---|---|
| **FE01** | **500mm 8mm Lead Screw Set (`CNC4759-A` — Active CAD Drive)** | `T8` 304 stainless, `Ø8 mm`, Pitch `2 mm`, Lead `8 mm`, `500 mm` + `1× T8` copper nut + `1× 8→5 mm` coupling + `2× KP08` pillow blocks · `Rs. 3,790.00` on [Duino.lk](https://www.duino.lk/product/500mm-8mm-lead-screw-copper-bearing-blocknut/) | **1 kit** | Cut screw to **`~202 mm`** (`298 mm` spare); mount `1× KP08` flush behind `Y = -245` wall (`-258.5..-245.5`), coupling behind wall (`-286.5..-261.5`), trim copper nut flange to `~9.8 mm` height |
| **FE02** | **Nema17 Stepper Motor 0.4Nm 4-wire Cable `17HS4401` (`Tronic.lk MT0382`, `Rs. 2,300.00`)** *(no worm gear; plugs straight into `CNC4759-A`)* | `42.3 × 42.3 × 40 mm` body, `0.4 Nm`, `1.7 A`, `1.8°`, **`Ø5 mm` D-shaft (`22–24 mm`)**, 4-wire bipolar | **1** | Bolted inline to `R46` rear bulkhead (`Y = -335.7..-295.7 mm`) inside protective side cheeks; drives `T8` screw coaxially via `8→5 mm` coupler |
| **FH01** | **688ZZ Miniature Ball Bearing** *(replaces front `KP08` inside low-clearance tray)* | `8 × 16 × 5 mm` | **1** | Clamped in `R49` front arch saddle (`Y = -76.5..-71.5 mm`, clears cargo cleanly) |
| **FH02** | **M5 × 16–18 mm Socket Cap Screws + M5 Hex Nuts** | `DIN 912` / `DIN 934` | **2 + 2** | Bolts rear `KP08` pillow block down onto `R46` rear shelf |
| **FH03** | **F683ZZ Flanged Miniature Ball Bearing** *(for Stage-2 `16T` spur pinions)* | `3 × 7 × 3 mm` (`Ø8.1 mm` flange) | **2** *(buy 4 spare)* | Seated inside `R54` / `R55` (`16T` horizontal `1:2` spur pinions on `R50`) |
| **FH04** | *GT2 Timing Belt (no longer needed — replaced by integrated `R47/R48` ↔ `R54/R55` ↔ `R51/R52` Rack-and-Pinion)* | — | **0** | Zero belt tensioning/stretch; positive gear tooth drive for `+240 mm` reach |
| **FH05** | **M3 × 10–16 mm Socket Cap Screws** *(4 top deck + 12 perimeter drop-skirt + 2 rear cradle + 2 rear `R44` spine)* | `DIN 912` | **20** *(buy 24)* | Clamp drop-on `R46` skirts to `R44` tray + bolt removable `R53-B01` cantilever tie plate under `R46`/`R44` |
| **FH06** | **M3 × 10–12 mm Socket Cap Screws + Nuts** *(for pulley pins & shaft pinch clamps)* | `DIN 912` / `DIN 934` | **8–10** | `F683ZZ` pulley axles (`4`) + split shaft clamp screws (`4–6`) |

---

### B. Steel Smooth Shafts (`tronic.lk` / `duino.lk`) — Replaces Expensive Linear Rails

| Part Name (Exact Store Listing) | Store & Code | Unit Price | Qty to Buy | Cut Plan & Exact Use |
|---|---|---|---|---|
| **`8mm Linear Shaft 500mm Smooth Shaft Rod`** | **Tronic.lk** (`MT0092`) | **Rs. 600.00** | **1 pc** *(500 mm total)* | Cut with cutoff wheel into **`2 × 165 mm`** Stage-1 guide rods (`330 mm` used) + **`170 mm` spare** for cross-axle / pins *(or buy **`2×` `8mm Linear Shaft 250mm` @ Rs. 300.00 each = Rs. 600.00**)* |
| *Optional:* **`300mm 6mm Smooth Shaft Rod`** | **Duino.lk** | **Rs. 590.00** | **2 pcs** *(or `0` if using metal/CF fork blades)* | Trim to **`2 × ~200 mm`** internal spine rods inside `R51`/`R52` fork tines to prevent tip sag under heavy `3–5 kg` loads |
| *Optional:* **`Shaft Collar Locker 8–20mm`** (`8 mm`) | **Duino.lk** | **Rs. 120–180** | **4 pcs** *(or `0` if using printed `M3` pinch clamps)* | Locks `Ø8 mm` guide rods axially if not held by printed `M3` split clamp blocks |

> **Machining Warning:** Smooth chrome linear shafts (`MT0092`) are **case-hardened** on the outside. **Do not try to thread them on a lathe or with a hand die** (use an abrasive cutoff wheel/angle grinder to cut to length, and buy a pre-made Lead Screw for any threaded drive).

---

### C. What NOT to Buy (Verified Over-Budget or Wrong Height)

| Store Item | Store / Price | Why Skipped |
|---|---|---|
| **`MGN12 100mm Square Linear Rail + MGN12C`** (`MT0047`) | Tronic.lk (`Rs. 4,500.00`) | **7.5×** more expensive than an `8 mm × 500 mm` smooth shaft (`Rs. 600`), and `100 mm` is too short for `+120 mm` Stage-1 stroke. |
| **`SFU1204 12mm Ball Screw + 1204 Nut`** (`CNC4715`) | Duino.lk (`Rs. 9,800–20,390`) | Industrial CNC size (`Ø12 mm` screw + bulky ball nut); far over-budget and exceeds our `12 mm` under-load height (`Z = 178..190 mm`). |
| **`SK8 8mm Linear Shaft Rod Support`** | Tronic.lk (`Rs. 270.00`) | Datasheet size is **`42 mm W × 32.8–33 mm H`** (`20 mm` shaft center height). Sticks `~23 mm` above our `12 mm` under-load bay (`Z = 178..190 mm`) and collides with cargo. |
| **`SHF8 8mm Linear Shaft Rod Support`** | Tronic.lk (`Rs. 250.00`) | Datasheet size is **`43 mm W × 24 mm H × 10 mm thick`**. Too tall for the tray front edge (`Y = -70 mm`) and leaves zero under-skid clearance (`SHF8`/`SK8` fit tall chassis/leg frames only). |
| **`MT0092` Hardened Shaft for Leg Swivel Axles (`M01`)** | Tronic.lk (`Rs. 600.00`) | **Leg pods (`M01`) need `2 × 69 mm` shafts drilled & tapped `M4 × 10` on both ends** — use **unhardened `8 mm` bright mild steel / silver steel** (`1 × 300 mm`), *not* case-hardened linear rod. |

---

### D. Saved Listing Reference — Heavy-Duty `T8` Kit (`CNC4759-A`, Duino.lk)

- **Product:** `500mm 8mm Lead Screw Pitch 2mm Lead 8mm Set Copper Nut + Coupling + Pillow Bearing Block`
- **SKU / Price:** **`CNC4759-A`** · **Rs. 3,790.00** · [Duino.lk Product Page](https://www.duino.lk/product/500mm-8mm-lead-screw-copper-bearing-blocknut/)
- **Confirmed Pack Specs:**
  - `1×` `T8` 304 stainless steel lead screw: **`Ø8 mm`**, **Pitch `2 mm`**, **Lead `8 mm`** (`8 mm` linear travel per revolution → only `15` turns for `+120 mm`), **Length `500 mm`**
  - `1×` Copper (`T8` Lead `8 mm`, Pitch `2 mm`) flange nut (`Ø22 mm` flange)
  - `1×` Coupling: **`8 mm to 5 mm`** bore
  - `2×` `8 mm` Pillow Bearing Blocks (`KP08`, `~28–29 mm` overall height)
- **Suitability & Required Fit Tweaks for the Forklift:**
  - **Pros:** Strong `Ø8 mm` stainless screw handles heavy `3–5 kg` loads cleanly and moves `~11×` faster per revolution than an `M4` (`0.7 mm`) screw.
  - **1. Cut Length:** Trim the `500 mm` screw down to **`~202 mm`** (`~298 mm` spare left over).
  - **2. Motor Match:** Included coupling fits the **`5 mm` `17HS4401` motor shaft** (`Tronic.lk MT0382`, selected for the CAD).
  - **3. Nut & Front Bearing Height (`12 mm` Under-Skid Limit):**
    - Trim top & bottom flats on the `Ø22 mm` copper nut flange down to `~9.8 mm` height so it clears the tray bay.
    - Mount `1× KP08` **only behind the `Y = -245 mm` rear wall** on `R46`; leave the second `KP08` unused and fit a **`688ZZ` (`8 × 16 × 5 mm`)** bearing in **`R49`**'s front arch saddle so cargo clears at the front edge.

---

## 2. 3D-Printed Parts (`models/components/objects_compartment/`, PETG, Batch `B01`)

Print settings: `0.2 mm` layers, `4` walls, `40 %` gyroid infill. Every STL is `1` solid and engraved `0.6 mm` deep per `CLAUDE.md` rules (`R49`, `R54`, `R55` are `id_only`).

| ID | Mark | STL File | Colour | Qty | Role & Key Dimensions |
|---|---|---|---|---|---|
| **R46** | `R46-B01` | `R46_forklift_deck_B01.stl` | `#26A69A` | **1** | Drop-on `180 × 175 mm` sloped deck, stop wall (`Y=-245..-238` w/ rack tail slots), unified `KP08` + wrapped `17HS4401` cradle, `3.2 mm` vertical corner & mid-edge skirts flush to `Z = 168.0` |
| **R47** | `R47-B01` | `R47_rail_left_B01.stl` | `#00897B` | **1** | Left fixed guide channel (`X = 132 mm`) + elevated bridged `MOD=1.0` Base Rack (`X = 156.5`, `Z = 183.9..187.9`) |
| **R48** | `R48-B01` | `R48_rail_right_B01.stl` | `#1B5E20` | **1** | Right fixed guide channel (`X = 228 mm`) + elevated bridged `MOD=1.0` Base Rack (`X = 203.5`, `Z = 183.9..187.9`) |
| **R49** | `R49` | `R49_front_688zz_arch_B01.stl` | `#80CBC4` | **1** | Front `688ZZ` bearing arch saddle (`Y = -77.5..-70.5 mm`, `Z <= 189.60 mm`), engraved on flat front arch face |
| **R50** | `R50-B01` | `R50_stage1_carriage_B01.stl` | `#43A047` | **1** | Stage-1 bridged H-carriage (`+120 mm` stroke), copper `T8` nut plate (`X = 180 mm`), vertical `M3` pinion bosses (`Y = -202 mm`) |
| **R51** | `R51-B01` | `R51_fork_tine_left_B01.stl` | `#F97316` | **1** | Left `4.0 mm` fork blade (`Z = 183.9..187.9 mm`), beveled nose wedge (`Y = -63 mm`), inboard `MOD=1.0` rack (`Y = -335..-78 mm`), `+240 mm` full reach |
| **R52** | `R52-B01` | `R52_fork_tine_right_B01.stl` | `#EA580C` | **1** | Right `4.0 mm` fork blade (`Z = 183.9..187.9 mm`), beveled nose wedge (`Y = -63 mm`), inboard `MOD=1.0` rack (`Y = -335..-78 mm`), `+240 mm` full reach |
| **R53** | `R53-B01` | `R53_under_tie_B01.stl` | `#2E7D32` | **1** | Removable under-tray rear cantilever tie plate (`Y = -288..-218`, flush at `Z = 155.70`); unbolts downward (`-Z`) so `R44` separates cleanly from `R46` |
| **R54** | `R54` | `R54_pinion_left_B01.stl` | `#EAB308` | **1** | Left horizontal `16T MOD=1.0` spur pinion (`rp = 8.0 mm`, `F683ZZ` center bore, `X = 148.5, Y = -202.0`), meshes `R47` ↔ `R51` at `1:2` |
| **R55** | `R55` | `R55_pinion_right_B01.stl` | `#CA8A04` | **1** | Right horizontal `16T MOD=1.0` spur pinion (`rp = 8.0 mm`, `F683ZZ` center bore, `X = 211.5, Y = -202.0`), meshes `R48` ↔ `R52` at `1:2` |

---

## 3. Required Cargo / Luggage Clearance Envelope

- **Clear Bay Width:** `175 mm` inside `R46` curbs (`184 mm` Shelf 1 rack opening).
- **Under-Cargo Skid Height:** **`12 mm` feet/skids** under outer edges (`X = 110..121` & `239..250 mm`) leaving an open **`Z = 178..190 mm`** fork tunnel on Shelf 1.
- **Robot Leg Z-Lift Needed:** **`+10 mm`** (`+7.8 mm` clean lift off Shelf 1; lowers unloaded onto `R46` outer wings during travel).
