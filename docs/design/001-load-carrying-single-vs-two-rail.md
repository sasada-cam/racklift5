# 001 - How the robot carries its load: single rail vs two rails (Squid style)

Status: **proposed** - 2026-11-04. Current concept: section 8h (compact Squid layout: two horizontal rails, swivelling hubs, telescopic arms).

## 1. The problem

The tray, basket and arm hang off the side of the rail. Gravity acting on that offset load produces a
**torque** that the robot must resist all the time, while it climbs, travels horizontally and turns.
The current v2 design resists it with a few small parts:

* the **hand gear and hand motor** (tray hangs ~230 mm to the side of the hand axle), and
* the **V-wheels on one rail** (wheels only ~40 mm apart across the rail, ~105 mm apart along it).

Nothing in the repo balances this load: there is no counterweight, ballast or battery compartment
(searched `generator/`, `cad/`, `docs/`, README, parts lists).

## 1b. Why the current v2 configuration works (what any change must not lose)

| Property | Physics | Effect |
|---|---|---|
| Low moving mass (~3.5 kg est.) | climb torque = m g r_pinion = ~0.36 N·m; energy = m g h | small climb motor, low energy per trip |
| In-slot rack and pinion | positive tooth engagement, no friction slip; rack protected inside the slot | no drift, load goes straight through the teeth |
| Worm-driven hand (1:600) | self-locking when friction angle > lead angle | holds the tray torque with power off |
| Hand keeps the tray level | slip starts at tilt = atan(mu) | basket cannot slide or tip during the 90 deg turn; no extra motor on the tray |
| Turn reacted by fixed junction pins | the force loop closes through the rack structure | wheels and rail only guide, they do not carry the turn torque |
| Passive hook pin | pull force limited by friction mu m g (~3.7 N for 0.75 kg, est.) | no gripper actuator, fewer failure modes, within a 20 kg servo |
| V-wheels in V-slot | 45 deg V self-centres and constrains all but the travel axis | stiff, low rolling friction, standard parts |
| One rail, 27 printed parts | fewer parts and actuators | cheap, easy to source locally |

These suit **light payloads**. The limits are the sideways tray load (~5 N·m on the hand gear at 0.5 kg) and the narrow
40 mm wheel stance (~80-110 N wheel load); see sections 2 and 3. Option D should keep low mass, the rack and pinion,
self-locking drives and the passive hook while fixing those two limits. The level tray comes free in D (the body does not
rotate), but the turn torque path through fixed pins would need redesigning.

## 2. Findings from the MuJoCo sim (`sim/`)

Static hold torque on the hand axle with the basket on the tray (`pixi run -e sim sim-headless --payload KG --counterweight KG`):

| Payload in basket | Counterweight | Hand-axle torque |
|---|---|---|
| 0.5 kg | none | 5.3 N·m |
| 2.0 kg | none | 8.8 N·m |
| 0.5 kg | 1.5 kg at 230 mm opposite | 0.08 N·m |
| 2.0 kg | 1.5 kg at 230 mm opposite | 5.3 N·m |

* About **2.35 N·m per kg of payload**, plus ~4.1 N·m for the empty tray + arm.
* The same moment is reacted by the rail ("body/rail reaction" in the sim output = hand-axle torque).
* The hand motor sees this divided by the 60T/12T gear ratio (5:1): ~1.1 N·m (11 kg·cm) at 0.5 kg, ~1.8 N·m (18 kg·cm) at 2 kg, before efficiency.
* A counterweight sized `~4.1 + 1.2 x max_payload` (N·m) balances empty and max payload to equal and opposite torque, but it adds climb load.

**Caveats.** Masses in `sim/model.py` are estimates. Motor torque figures are *not* known (no datasheet
for the 1:200 / 1:600 JGY-370 versions was found; retail listings conflict). Fetch *dynamics* in the sim
are not yet reliable (basket landing position varies, peak torques hit actuator limits) - only the
static numbers above are trusted, and they match hand calculations.

## 3. Why one rail is weak, and why two rails fix it

A torque is reacted as a **couple**: `force = torque / distance between the two supports`.

| Situation | Axis of the moment | What resists it today | Distance |
|---|---|---|---|
| Climbing (vertical rail) | across the rail | wheels along the rail | ~105 mm |
| Horizontal travel | the rail's own axis (tray hangs ~120 mm in front of the rail) | wheels either side of the rail | **~40 mm** |

Hand estimate (not simulated), ~2.5 kg hanging load, lever ~0.12 m, torque ~3 N·m, single rail with ~40 mm wheel spacing:
**~75 N** on the wheels.

Spreading two carriages further apart *along the same rail* (option A) helps the climbing case only.
It does **not** change the 40 mm that resists the rail-axis moment during horizontal travel.

> **Correction (2026-11-04, after `sim/legs.py`).** An earlier draft of this note said two rails 300 mm apart would cut the
> wheel force to ~10 N per rail. That ignored that each clamp also carries its half of the weight **at the leg's stand-off
> from the rail**, which is a moment on the clamp's own wheels. The two-rail/two-leg layout only helps if the clamp has a
> **wide wheel stance across the rail** and the legs **stand off a short distance** (see section 5b).

## 4. Reference: BionicHIVE "Squid" (from photos only, no specs seen)

* Two separate leg units, each with its own wheels and rotating joint; tray + arm hang between them.
* Rails run in parallel pairs (one above, one below the tray).
* At a junction one leg turns while the other stays locked on a fixed rail, so the robot is never fully unclamped.
* Motors/electronics sit in the leg units, close to the rails. Mass at the rail adds no torque, but it is **not** a
  counterweight either: only mass on the opposite side of the load cancels torque.

## 5. Options

| | Current v2 | A: two legs, one rail (~300 mm apart) | B: Squid style, two parallel rails | C: counterweight on current design |
|---|---|---|---|---|
| Climbing stability | weak | better | good | unchanged wheel couple |
| Horizontal-travel stability | weak (~40 mm) | **still weak** | good | **still weak** |
| Hand gear / hand motor load | high | lower | low | low for design payload |
| Extra rail / turntables | none | none | 2x rails, 2x racks, 2x turntables per junction | none |
| Extra moving mass | none | small | motor per leg | +1.5..3 kg to lift |
| Turning safety | all-or-nothing | all-or-nothing | one leg always locked | all-or-nothing |
| Simultaneous leg rotation | n/a | n/a | possible; must be synchronised (same angle) - default to one leg at a time | n/a |
| Build effort | none | small | large | small |

## 5b. Option D: two fixed-length legs, powered hinge angles, in-line rails (chosen direction for the next revision)

Derived from Squid photos (not specs): the two legs are **fixed length**; each has its own **powered hinge** that sets its angle
(so span and stand-off change without telescoping), plus the turntable rotation for vertical <-> horizontal.

* **Rails:** one vertical rail per bay, legs in line along it; **two horizontal rails** spaced by the distance D between the leg
  ends on the body; a turntable junction at each of the two horizontal rails.
* **Climb:** span between clamps = `D + 2 L cos(phi)` (long span for stability, short near the rail ends).
* **Travel:** span = D (horizontal rail spacing).
* **Powered hinge:** a self-locking worm drive (e.g. JGY-370) holds the angle without power.

Static model `sim/legs.py` (`pixi run -e sim python sim/legs.py`; assumptions at the top of the file: L = 200 mm, D = 300 mm,
body 1.3 kg, tray + arm + basket 1.5 kg + payload, tray 230 mm sideways and 100 mm beyond the leg ends, lower drive carries half
the weight). Worst clamp wheel load, payload 1 kg:

| Clamp wheel stance across the rail | phi = 90 deg (stand-off 200 mm) | phi = 60 deg (173 mm) | phi = 40 deg (129 mm) |
|---|---|---|---|
| 40 mm (as v2) | 94 N | 81 N | 60 N |
| 120 mm (wide clamp) | 31 N | 27 N | 20 N |
| *current v2, single rail* | *82 N climb / 113 N travel* | | |

Other results: leg hinge torque 0.3-3.8 N·m at 1 kg payload and 0.7-5.3 N·m at 2 kg, depending on leg angle and direction;
climb-pinion torque 0.2 N·m at 1 kg; axial drive couple on the two horizontal rails 0.2-19 N each at 1 kg.

What this says:
* Two legs / two rails with a **narrow clamp barely beat v2** (60-94 N vs 82-113 N). The gain comes from a **wide wheel stance
  across the rail** (3x lower) and a **small stand-off** (low phi).
* The powered hinge is only a few N·m, within reach of a worm motor, but that is an estimate; motor ratings are still unknown.
* Mass and geometry in `sim/legs.py` are estimates. This is a static model: no accelerations, flex or play.

## 6. Recommendation

* Payload ~1 kg or less, accepting weak horizontal-travel margin: A (+ optionally C) is the cheap path.
* Anything heavier, or if stability matters: **D** (two legs, powered hinge angles, in-line rails) **with a wide-stance
  clamp (~120 mm across the rail) and short stand-off**. Full two-of-everything rails (B) is not needed.
* Do not choose A expecting it to fix horizontal travel.
* Do not expect two rails alone to fix wheel loads: clamp stance and stand-off decide it.

## 7. What B (and D) change in the rack and rails (to confirm in design)

For D only the horizontal rails are doubled; the vertical rail per bay stays single (legs in line along it).

* **Two of every rail:** a vertical pair per bay and a horizontal pair between bays, each with its own printed rack strips.
* **Junctions:** two turntables per junction pair, aligned to the leg spacing; a leg turns on its own turntable.
* **Cupboard (slotted-angle rack):** the shelving itself can probably stay. Rail centres at +/-172 mm already sit on the
  posts' front flanges (`XR = XI + ANG/2`; the archived v1 two-rail lift used this). What changes is the mounting:
  rail bridges (J06), base plates and spacers (J05), the junction geometry, and mounting points for the horizontal pair.
* **Open:** whether the horizontal pair (two heights, ~300 mm apart) clears the shelf levels (175 mm pitch) and the arm reach path.

## 8. Related finding: baskets cannot be put back by the robot

The hook pin tip is ~30 mm below the basket rim and the arm reaches only ~70 mm past the shelf front, so the pin cannot get in
front of a basket on the tray to push it back. The current design **fetches** baskets to an operator; returning them is manual.

## 8b. v3 first build: rail pairs (2026-11-04, superseded by 8c)

Chosen: option D with **two 20x20 rails per direction** (clamp rides a rail pair 120 mm apart). Source `generator/params_v3.py`,
`generator/assembly_v3.py`; viewer `pixi run -e sim sim`; CAD `pixi run export-v3` -> `models/v3/`.

| | value |
|---|---|
| Leg | fixed 150 mm, powered worm hinge at the clamp, passive pivot on the body, pivots 160 mm apart |
| Spread (climb, pick, travel) | phi 15.5 deg, clamp span 449 mm, body in the v2 position (tray fits the shelves) |
| Folded (turn) | phi 90 deg, span 160 mm, body 110 mm further out, both clamps on the 300 mm turntable piece |
| Clamp | wheels on the outer faces of the rail pair: 160 mm across, 90 mm along |
| Junction | turntable rail-pair piece at z = 185, horizontal rails z = 125 / 245 |

Static loads (`pixi run -e sim legs --v3`), 1 kg payload, spread legs: wheel load 9 N across / 57 N along the rail climbing,
5 N / 30 N travelling; leg hinge 1.5-2.2 N·m; climb pinion 0.2 N·m. Folded (turning only): up to 33 N / 59 N, hinge 5.3 N·m.
v2 for comparison: 82 N climbing, 113 N travelling. The clamp wheelbase along the rail is now the limiting dimension
(50 mm gave ~100 N; 90 mm gives 57 N).

Open in v3: shelf 1 is behind the horizontal rails; vertical rails extend 160 mm above the rack; junction turn torque
reaction (v2 used fixed pins + the hand) not yet redesigned; no clearance check or FEM yet.

## 8c. Tried and dropped: one rail in front of the bay, tray centred on the turn axis (2026-11-04)

> Dropped: it moved the rails in front of the rack and the travel rail to the top. The rack and rails are to stay as in
> v2; only the robot body changes (section 8d).

Why the change: the second rail per direction added parts (two racks, rail pairs that must stay parallel or the clamp
jams), and the static model showed it barely helps with the legs spread (the worst load, 57 N along the rail, did not
change). The real cause of the large wheel loads was the tray hanging 230 mm **beside** the rail. Squid puts the
compartment on the same axis the rail rotates about, so a turn spins the load in place.

What changed (source `generator/params_v3.py`, `generator/assembly_v3.py`):

| | value |
|---|---|
| Rail | one 20x80 V-slot per direction, wheels on the two narrow faces (stance ~100 mm across, 90 mm along) |
| Vertical rail | up the **centre** of each bay, centre 438 mm in front of the rack, robot rides on the **back** of the rail |
| Tray | hangs centred on the hand axle (tray top 55 mm below it); axle = turn axis = turntable axis at a junction |
| Horizontal rail | along the **top** (z = 1090) above the rack, hung from an overhead beam (z 1400) on post extensions |
| Junction | turntable piece +/-150 mm, plus a 280 mm stub above it so the legs can spread right at the junction |
| Legs | unchanged: 150 mm, pivots 160 mm apart, spread phi 15.5 deg (span 449), folded 90 deg (span 160) |
| Shelves | 2-5 (shelf 1 would need the lower clamp below the floor) |

Why the rail is in front and the travel is at the top:
* With the tray centred on a rail in front of the bay, the rail would sit between the basket and the tray if the robot rode on
  its front. So the robot rides on the back.
* Folding the legs moves the body 110 mm toward the rack. That space is only free above the rack, so the junctions and the
  horizontal rail are at the top.

Static loads (`pixi run -e sim legs --v3`, tray + arm + basket centre of mass assumed at y = -150 mm):

| 1 kg payload | v2 | v3 rail pairs (8b) | v3 one rail, centred tray |
|---|---|---|---|
| Climbing, worst wheel | 82 N | 57 N | **38 N** |
| Travelling, worst wheel | 113 N | 30 N | **40 N** |
| Hand / turn motor (keeps the tray level) | 5.3 N·m | 5.3 N·m | **~0** (load on the axle) |
| Leg hinge | - | 1.5-2.2 N·m | **4.0 N·m** (5.7 at 2 kg) |

Trade-offs:
* **Leg hinge torque goes up**, because the tray now stands 215 mm beyond the body pivots. Two ways to cut it: wider body
  pivots (D) or a shorter tray. It needs the real hinge motor rating.
* **Frame:** stands 1.44 m high, and the rails are 440 mm in front of the rack (aisle space).
* **Stiffness not checked:** the free vertical rail span (~0.8 m) and the overhead frame.

Collision check: `pixi run clearance-v3` checks 33 poses (every pick, turns, junction spread/fold, travel). Clear apart from
one 4 mm3 mesh-faceting overlap of the pinion in the rail slot, which is listed as a skip in the script.

## 8d. v3 as built now: v2 rails, Squid-style two-hub body (2026-11-04)

Decision: keep the v2 rack, rails, in-slot racks and turntable junctions **unchanged**; re-arrange only the robot body like
the Squid photos. The motors sit in two hub units on the rail, and the tray sticks out sideways to the bay.

| | v2 | v3 |
|---|---|---|
| Body | one chassis, two gantry plates 65 mm apart | upper + lower hub on the same rail, 100 mm apart, joined by a column |
| Climb drive | 1 pinion + 1 JGY-370 | 1 pinion + 1 JGY-370 **per hub** (both drive) |
| Wheels along the rail (outer) | 105 mm | 165 mm |
| Tray-level (turn) motor | below the hand axle | beside it on the tray side, clear of the hubs and the junction torque pins |
| Rails, junctions, hand, tray, arm | - | unchanged |

Constraints that set the hub size: in horizontal travel one hub sits on the tray side, and the tray starts 100 mm from the
rail, so everything on the body stays within +/-97 mm along the rail. That also fits the +/-150 mm turntable piece and keeps
the upper wheels below the 1000 mm rail top at the top shelf. The v2 67-pose collision check passes (`pixi run clearance-v3`).

Static wheel loads (`pixi run -e sim legs --v3`), 1 kg payload:

| | v2 | v3 |
|---|---|---|
| Climbing | 82 N | **52 N** |
| Travelling | 113 N | 113 N (same rail and wheel stance) |

Hand-axle torque is unchanged from v2 (~5.3 N·m at 0.5 kg), because the tray is still 230 mm beside the rail.

Not in this version: powered leg angles and turning one hub at a time. The v2 turntable carries both hubs together.

## 8e. Tried: walking legs with swivelling hubs on the v2 rails (2026-11-05) - does not fit

Idea (Squid): two legs pivot on a centre joint, each hub swivels 90 deg on its leg end, a level beam carries the tray and a
counterweight. At a junction one hub stays clamped on a fixed rail while the other swivels with the turntable piece.
Draft: `generator/params_v3_legs.py`, `assembly_v3_legs.py`, `clearance_v3_legs.py` (82-pose check, 15 poses collide).

Why it does not fit the v2 rails:
* In v2 horizontal travel, the tray and basket (122 mm tall) ride in the gap between the horizontal rail (top at JZ + 10) and
  the start of the vertical rails (JZ + 151), with 3 mm to spare. Nothing of the robot is under the tray.
* With legs, both hubs sit on the horizontal rail, symmetric about the centre joint, so the right hub is under the tray.
  A hub on a 20x20 rail is at least +/-33 mm high (wheels, plate). Hub + tray + basket needs ~160 mm, and the gap is 141 mm.
* Lifting the tray over the hub puts the basket into the next bay's vertical rail, rail bridge and junction.
* The walk needs one hub fully off the 300 mm turntable piece while the other is on it. That forces legs of at least
  ~150 mm, which pushes the right hub further under the tray.

What would make it fit (all change the v2 rails): longer turntable pieces (about +/-250 mm) with longer legs and vertical rails
~200 mm taller, or Squid-style curved junction rails instead of turntables.
Without walking, the parts that do fit the v2 rails are the counterweight beam, the linear slide and the spring-loaded pin.

## 8f. v3 additions that fit the v2 rails: counterweight, linear slide, spring-loaded pin (2026-11-05)

Added to the two-hub robot of 8d (rails still v2, turning still on the v2 turntables):

* **Counterweight:** a battery / electronics box on the hand, opposite the tray, so it stays level with it. Estimate: 2.0 kg at
  260 mm = 5.1 N·m, which balances the tray at ~0.4 kg payload. `CW_MASS` in `params_v3.py`; set it from the real battery.
* **Linear slide instead of the SCARA arm:** one N20 motor, GT2 belt and a 20x20 rail, 150 mm above the tray on the bay
  centre line.
  * The rail stops at y = -70 so it passes in front of the vertical rails in horizontal travel.
  * The pin sits on a 130 mm arm in front of the carriage.
  * Stroke: pin y -240 to +60.
* **Spring-loaded hook pin:** chamfered tip, 35 mm spring travel, set 22 mm beside the slide rail so it can lift. An SG90 servo
  locks it.
  * Unlocked: it rides up over a basket wall and drops behind it.
  * Locked: it pulls or pushes.
  * This removes the limit of section 8. The robot now pushes baskets back:
    1. With the basket on the tray, the pin rides out over the near wall.
    2. The servo locks it, and the slide pushes the basket onto the shelf.
  * The 35 mm robot lift before hooking is no longer needed.

Collision check `pixi run clearance-v3`: 97 poses, 0 collisions. That covers every shelf, pin over the front wall, hooked,
pulling, pulled, pin out over the near wall, pushing, pushed back, turns at both junctions, travel and climb.

Loads (`pixi run -e sim legs --v3`, estimates):

| payload | hand torque v2 / v3 | climbing wheel load v2 / v3 | travelling wheel load v2 / v3 |
|---|---|---|---|
| 0 kg | 4.1 / -1.0 N·m | 60 / 6 N | 83 / 222 N |
| 1 kg | 6.4 / +1.3 N·m | 82 / 8 N | 113 / 252 N |
| 2 kg | 8.8 / +3.7 N·m | 103 / 22 N | 143 / 281 N |

**Crane layout (2026-11-05, latest):** the counterweight and the slide support now sit on one straight beam along x
(counter-jib + jib, like a tower crane), on a mast from the hand, with a 90 deg elbow into the slide rail. The front post and
tray extension are gone, so the robot ends at y = -360 instead of -410. The slide rail is 290 mm, the pin arm 125 mm, and the
pin stroke is still -205..+55. Collision check: 97 poses, 0 collisions.
Cost: the counterweight now sits ~280 mm in front of the rail instead of ~140 mm. With the travel load computed from that real
stand-off, travelling wheel loads at 1 kg are ~252 N (crane) vs ~180 N (counterweight near the hand) vs 113 N (v2, no
counterweight). Climbing and hand torque are unchanged (they depend only on the x lever).

Trade-off: the counterweight almost removes the hand torque and the climbing wheel load. But in horizontal travel its weight adds to
the moment about the rail axis, which the 40 mm wheel stance of a 20x20 rail must carry, so travel loads roughly double with the crane layout. A lighter
counterweight (battery only) or a wider rail (20x40) would ease that.

## 8g. (superseded by 8h) Squid layout with crane and sliding counterweight: Squid layout - two horizontal rails, swivelling hubs, auto-balancing counterweight (2026-11-05)

This is the layout intended from the start ("like Squid"):
* **Travel:** two horizontal rails at the bottom (z 140 and 330), one hub on each.
* **Climbing:** both hubs on one vertical rail per bay.
* **Crossings:** each crossing has a small turntable piece (+/-94 mm). A hub swivels 90 deg together with its piece, while the
  other hub stays locked on its rail.
* **Body:** it never rotates. The v2 hand turn motor, torque pins and 300 mm turntables are gone.

Problems found on the way and how they were solved:
* **The tray must stay in front of the rails (y < -60).** With the v2 tray reaching to y -3, the basket hit the upper
  horizontal rail when climbing past it, and the crossing turntables when travelling. Fix: the tray ends at y -70, and a
  **drawbridge lip** (hinged plate, SG90) folds down onto the shelf during a transfer. Folded up, it is the basket stop.
* **The hubs swivel behind the column (y > -94)**, inside a ~56 mm radius. The column, swivel motors and crane sit in front of
  that.
* **Crane and counterweight:**
  * The counterweight is a battery sled on a self-locking M8 lead screw along the counter-jib.
  * A bar load cell (HX711) under the tray bracket weighs the payload.
  * The sled moves to x = -(4.1 + 2.35 m) / (2.0 g) (estimate): -209 mm empty, -449 mm at 2 kg.
  * This copies the variable-position counterweights of mobile cranes (Manitowoc VPC, Liebherr VarioBallast). A gyro cannot
    sense the imbalance here, because the body is rigidly held by two rails and never tilts. The load cell measures it
    directly.

Loads (`pixi run -e sim legs --v3`, 1 kg payload, estimates):
* The in-plane moment of the tray is balanced by the sled, leaving ~0 N on the side wheels and the drive pinions.
* The stand-off moment (everything hangs in front of the rails) is now a couple between the two hubs, 190 mm apart, instead
  of across a 40 mm wheel stance: **~59 N** (v2: 82 N climbing, 113 N travelling).
* The far-forward counterweight is most of that. A heavier sled closer to the column would reduce it.

Collision check `pixi run clearance-v3`: 100 poses, 0 collisions. That covers:
* every shelf: pin over the wall, hooked, pulling over the lip, pulled, pin out, pushing, pushed back;
* both hub swivels at both bays;
* climbing past the upper rail;
* travel with the sled at both ends of its screw.

Rail changes vs v2:
* a second horizontal rail;
* four small crossing turntables instead of two 300 mm ones;
* vertical rails 80 mm taller (upper hub at the top shelf).

## 8h. v3 as built now: compact Squid layout (2026-11-05)

Why: the 8g robot was large. It had a 0.9 m crane beam, a 2 kg sled 360 mm in front of the rails, plywood crossing plates
and an overhead slide. Most of its 59 N worst wheel load came from the counterweight hanging far forward, so it was also
unbalanced seen from the top.

Changes:
* **Rails:**
  * The lower horizontal rail moved from z 140 to 70, so the hubs are 260 mm apart instead of 190. Every couple is divided by it.
  * The crossing turntable pieces are +/-58 mm on 50 mm bearings with small printed brackets. The plywood plates are gone.
* **Picker:** two 2-stage telescopic arms (180 mm stages, drawer-slide style) in the 23 mm gaps between basket and rack posts,
  with front and rear flip fingers on micro servos. Real-world model: the telescopic load handlers of warehouse shuttles
  (e.g. Dematic Multishuttle, Knapp OSR).
  * Pull: front fingers behind the far wall (8 mm space before the rack's rear flange), then retract.
  * Push: the rear fingers are 3 mm behind the near wall when stowed. Flip them in, then extend.
  * The basket never drops into the 110 mm gap: its centre of gravity stays over the shelf until its front edge is on the tray.
  * This removes the crane beam, elbow, overhead slide, spring pin, lock servo and drawbridge lip.
* **Battery box:** a 1 kg battery box is fixed beside the column (120 mm out, 80 mm in front of the rails), as a passive
  counterweight. The moving sled, lead screw and screw motor are removed. The load cell stays for the payload and an overload check.
* **Tray:** 205 mm deep, so the robot ends ~275 mm in front of the rack (8g: ~445 mm).

Loads (`pixi run -e sim legs --v3`, estimates), worst wheel or pinion force:

| payload | v2 | 8g (crane + sled) | 8h compact |
|---|---|---|---|
| 0 kg | 83 N | ~53 N | 13 N |
| 1 kg | 113 N | ~59 N | 17 N |
| 2 kg | 143 N | ~64 N | 24 N |

Collision check `pixi run clearance-v3`: 103 poses, 0 collisions. That covers:
* every shelf: arms out, fingers in, pulling, pulled, stowed, rear fingers in, pushing, pushed back;
* hub swivels at both bays;
* climbing;
* travel.

### 8h addendum: four legs?  Live balance view

**Four legs:** a second column with two more hubs on the next bay's vertical rail. The v2 rails are spaced exactly one bay
apart, and that spacing is a whole number of rack teeth, so the racks are in phase.

Gain: the tray hangs between the columns like a bridge deck, so the twist ~0 and the forward couple is shared, ~9 N per hub
at 1 kg instead of 17 N.

Costs:
* 4 climb + 4 swivel motors;
* ~1 kg more mass and a ~0.5 m wide frame;
* one extra vertical rail past the last bay;
* the two columns must climb in step on perfectly parallel rails, so one column needs sideways float.

Not adopted: two legs already give ~6x lower loads than v2. Reconsider for payloads of 4-5 kg.

**Live balance view:** the sim (`pixi run -e sim sim`) draws the centre of gravity, the weight, and the rail forces on both
hubs, from a static estimate of the current pose. Task 11 pulls the heaviest basket (1.6 kg) slowly and switches the
battery's effect off and on:

| | twist | forward tip |
|---|---|---|
| empty tray | 1.1 N·m (4 N per hub) | 2.4 N·m (9 N per hub) |
| loaded, battery on | 5.4 N·m (21 N per hub) | 4.5 N·m (17 N per hub) |
| loaded, battery off | 6.5 N·m (25 N per hub) | 3.7 N·m (14 N per hub) |

The battery cuts the twist but adds a little forward tip, because it sits 80 mm in front of the rails.

## 9. Needed before deciding

1. Rated and stall torque of the JGY-370 1:200 (climb) and 1:600 (hand) actually bought, plus the 20 kg servos.
2. Real masses: chassis, hand + tray + arm, basket, and the maximum payload to support.
3. Target payload range (min / max) and required climb speed (the sim climbs at 0.12 m/s; the real 1:200 motor is ~0.024 m/s).
4. Rail spacing that fits the shelf pitch and arm reach.
