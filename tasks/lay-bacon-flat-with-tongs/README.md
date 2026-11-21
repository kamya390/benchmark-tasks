# Lay three bacon rashers flat on a sheet pan with tongs, side by side, none folded or overlapping

Three limp rashers lie shingled in a deli tray beside a quarter sheet pan. The robot takes a pair of spring
tongs off their stand, picks each rasher up by one end with them, and lays it flat on the pan floor by touching
its free end down and trailing the rest out straight, so that it neither folds on itself nor lands on its
neighbour. Then it puts the tongs back and lets go.

## What it tests

Laying a limp strip flat through a hand tool. A strip held by one end and simply lowered folds over on itself;
it only ends up flat if the tail is put down first and the held end is drawn away along the strip's own line
while it comes down. The robot does this through tongs: its gripper squeezes two sprung arms, and the arms'
tips hold the rasher, so grip force, tip position and tip pitch all reach the food second-hand. There are no
clips, pegs, slots or guides: the pan is a bare flat floor.

Laying bacon strips side by side on a sheet pan is a recognizable home-cooking task when preparing bacon in
the oven. The category is Societal relatability. Folded or overlapping strips cook unevenly, so each limp
strip must be laid out flat. The physical challenge is preserved with dry silicone stand-ins and spring
tongs. Saveur's oven-bacon guide describes laying the strips in a single layer without overlap.
[Oven-bacon preparation](https://www.saveur.com/food/how-to-cook-bacon-in-oven/)

## Run

```sh
inspect-robots run --embodiment yam_arena -E task=lay_bacon_flat_with_tongs.py \
  --instruction "Lay the bacon out on the sheet pan for the oven, using the tongs." \
  --scorer task_score --max-steps 2000
```

Scripted demo: add `--policy yam_scripted -E object_jitter=0 -E control_hz=20`.
Offline checks (no simulator): `python3 -m unittest test_scoring.py`

Run limit 2000 steps (100 s at 20 Hz); the demo reaches 1.0 after about 1530 steps. Seeds: none, the layout is fixed.

## Files

- `lay_bacon_flat_with_tongs.py`: scene, hinged rashers and tongs, per-step pose feed, `score()`, `success()`,
  `demo()`.
- `scoring.py`: pure tracker and score ladder (NumPy only).
- `test_scoring.py`: each rung, each full-score condition, folded, raised, overlapping, over-the-rim, gripper
  touch, not carried in the tongs.
- `bodykit.py`: our own composite-body and joint helper.

## Setup

x is the distance ahead of the line through the two arm bases, y is left (+) or right (-) of the centre line.

| Object | Size, material | Where |
|---|---|---|
| Quarter sheet pan | bare aluminium, 23 x 33 cm outside, 2.5 cm rim, flat floor 20 x 30 cm (e.g. Nordic Ware Naturals Quarter Sheet) | long side left-right, centre at x 31.5 cm, y +5 cm; taped down |
| Deli tray | rigid black plastic, 16 x 23 cm outside, 15 x 22 cm inside, 2.5 cm deep | long side left-right, 2 mm from the pan's right-hand short side, centre at x 31.5 cm; taped down |
| 3 bacon stand-ins | 250 x 35 mm strips of 1/8 in (3.2 mm) 40A food-grade silicone sheet (e.g. USA Industrials BULK-RS-S40CLRFDA-10), about 30 g each, dry | lengthwise in the tray, shingled 22 mm apart (centres at x 29.3, 31.5, 33.7 cm, farthest on top), left ends across both rims and 20 mm out over the pan floor |
| Spring tongs | stainless, 24 cm, flat 16 mm ends, no lock, about 6 cm open at the tips, about 90 g (e.g. Winco UT-9) | on the stand, one arm above the other, hinge at x 47.5 cm, y +25 cm, tips pointing right |
| Tongs stand | two 22 x 12 x 75 mm posts 185 mm apart with 8 mm side lips, on a 60 x 250 mm base | behind the pan, under the tongs; fixed |

## Score

A rasher is **laid** when, at the end of the run:

- all of it is on the pan's 20 x 30 cm flat floor;
- no point of it is more than 5 mm above the floor (the strip is 3 mm thick, so any folded or stacked part is
  at least 6 mm);
- its two ends are at least 235 mm apart (of 250);
- seen from above, it covers no part of another rasher (edges may touch);
- it was carried in the tongs (both jaws on it while it was at least 6 cm up), and no gripper ever came within
  10 mm of it. Both are latched for the whole episode.

| Score | Condition |
|---|---|
| 0.25 / 0.5 / 0.75 | one / two / three rashers laid |
| 1.0 | three laid, tongs back on the stand within 3 cm of where they started, both gripper centre points at least 4 cm from the tongs and every rasher, everything still for 1 s |

| Simulator check | Real-trial check |
|---|---|
| on the floor, ends 235 mm apart, no overlap | one photo from directly above with a ruler in the frame |
| nothing above 5 mm | a 5 mm hex key lying on the pan floor under a straightedge: the straightedge passes over every strip |
| carried in the tongs; gripper never within 10 mm | the two camera videos |
| tongs within 3 cm of start | outline of the tongs drawn on the stand's base |
| grippers 4 cm clear, 1 s still | the videos |

## Stand-in calibration

Raw bacon cannot go on a shared rig, so the rashers are strips of a named silicone sheet. What matters for
this task is how limp the strip is and how it grips the pan.

- **Bending.** For 3.2 mm 40A silicone, E is about 1.7 MPa (Gent's hardness relation), so a 35 mm wide strip
  has EI = 1.6e-4 N m2 and weighs 1.26 N per metre. The large-deflection beam solution for an 83 mm overhang
  (load parameter wL3/EI = 4.5) gives a tip drop of 35 to 40 mm. The simulated chain's hinge stiffness
  (8e-3 N m/rad) was set to match: in the `BACON_STAGE=droop` run, a rasher with two links (83 mm) over a
  block edge hangs with its end about 45 mm below the block top.
- **Friction.** Dry silicone rubber on bare metal is tacky, about 1.0 in published tables; the simulator uses
  0.9 static and 0.8 sliding between strip and pan, and 1.0 strip on strip.
- **Not measured.** These numbers come from published data and calculation. We have not bought and measured
  the sheet. The reset notes therefore carry two acceptance checks to run once per sheet (overhang droop
  30 to 50 mm; no sliding on the pan below 30 degrees of tilt).
- **Sensitivity.** The scripted demo was re-run with the strip's friction set to 0.6, 1.0 and 1.3
  (`BACON_MU`); it scored 1.0 each time, so the lay does not hinge on the exact friction value.
- **Why trailing is needed.** In the `BACON_STAGE=naive` run the same pick is followed by lowering the held end
  straight down over the lane and letting go. The strip buckles, swings round under the jaws and comes to
  rest across the pan with one end up on the rim: not flat, score 0.

## Fidelity

Rigid-body Isaac Sim/PhysX, 240 Hz physics, 20 Hz control.

- Each rasher is six rigid 41.7 x 35 x 3 mm links on five sprung, damped hinges. It bends only across its
  width and only at five lines; it cannot twist, stretch or curl sideways.
- Nothing about the rashers is scripted. They are dropped 2 to 14 mm onto the tray at reset and drape over the
  rims by themselves; folding, dragging and overlapping come from contact physics.
- The tongs are two rigid arms on a sprung hinge (3.5 N at the grip point to close). The arms have 3 mm
  collision thickness; real ones are about 1 mm.
- The pan's sloped rim is a 15 mm wide vertical wall. Pan, tray and stand are fixed (taped on the real bench).
- Raw bacon's stickiness and grease are not modelled.

## Demo

Right arm only. It reaches forward with its fingers one above the other, slides them over and under the tongs
on the stand and takes them about 10 degrees open. For each rasher, top of the shingle first, it slides the
lower jaw under the end that sticks out over the pan, pinches, peels the strip up off the stack, draws it out
over the pan along its lane so the tail drops onto the floor first, and lowers the held end while still
moving away, then opens the tongs and draws the lower jaw out from under the strip. The wrist roll sets the
tongs' pitch. Finally the tongs go back on the stand.

## Reset

Put the three strips back in the tray on the paper template (three centre lines and the 20 mm overhang
line) and the tongs back on the stand, tips right. Nothing is consumed and nothing wet or greasy touches the
rig.

## Assets and licence

Procedural primitives and our own code only (`bodykit.py` is our helper). No third-party meshes, textures or
scans.

## Response to review of TK-7L9VHX

Changed the category to Societal relatability and explained the recognizable home-cooking operation.
Removed the occupation-count argument from the context and README.
