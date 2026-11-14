# Square up a racked cabinet face frame on the bench until its diagonals match

Instruction given to the policy: `This face frame is out of square. Make it square, with both diagonals the same length, without moving it off the blue corner marks or lifting it, then let go.`

Files: `task.json`, `square_racked_face_frame.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`, the
sim `controls()` and an offline self-test), `mechkit.py` (this repository's hinge/friction helper, unchanged), `README.md`.

## The job

Cabinet face frames are dry-assembled from stiles and rails and checked for square by measuring both corner-to-corner
diagonals before glue-up; if they differ, the frame is racked along its long diagonal until they match. This is
regular work for carpenters (O*NET 47-2031, 889,700 US jobs; Core: "Verify trueness of structure, using plumb bob and
level.") and cabinetmakers (51-7011, 81,800); see Sources. The task is that dry-fit check: no glue in the trial. The
skill tested is two-handed: a light frame lying on a bench slides as a whole when one corner is pushed or one rail is
dragged, so the hands must work against each other (squeeze the long diagonal, or hold one rail and shift the other),
and they must stop within 2 mm of square, because nothing stops the frame at square.

## The frame (full size)

| Part | Spec | Source / derivation |
|---|---|---|
| Frame | face frame of an 18 in wide x 12 in tall wall cabinet: 457 x 305 mm outside | stock wall-cabinet size W1812 (cabinet maker's catalogue, Sources) |
| Stock | 3/4 x 1 1/2 in (19 x 38 mm) | Woodsmith face-frame guidance: 3/4 in thick, stiles at least 1 1/2 in |
| Stiles | 305 mm, full height, along x | |
| Rails | 381 mm between the stiles, along y | 457 - 2 x 38 |
| Wood | yellow-poplar, 470 kg/m3: 0.129 kg per rail, 0.103 kg per stile, 0.47 kg in all | specific gravity 0.42 at 12 % MC (Wood Handbook, Table 5-3a), x 1.12 for the water |
| Joints | one beech loose tenon 5 x 18.8 x 30 mm per joint, glued in the rail end (tight mortise), dry and snug in thickness in the stile mortise, which is 15 mm deep and cut at the joiner's middle width setting: 6 mm longer than the tenon is wide (3 mm of play at each end) | joiner maker's manual: mortise depth half the tenon's length; middle position 6 mm wider than the tenon; tenon 5 x 18.8 x 30 mm (maker's spec) |
| Bench | bare oak bench top, frame on it at mu 0.35 (the sim's station table is oak, at PhysX's default friction 0.5; the frame's 0.2 averages with it to 0.35) | wood on wood, clean and dry: 0.25-0.5 |

### Does the joint allow the rack? (clearance check)

Racking turns each tenon in its 15 mm deep stile mortise about the rail's shoulder. With about 14 mm of the 30 mm
tenon in the stile, its far end swings 14 sin(phi) along the mortise: 0.66 mm at the 2.71 deg start rack. The
middle-setting mortise leaves (24.8 - 18.8) / 2 = 3 mm at each end, so the start rack uses less than a quarter of the play (the
rotation alone would need 12 deg to use it up). A tight mortise (zero play) could not rack at all: the tenon would
have to crush 0.66 mm of beech. The shoulder bears on one corner and opens at the other by 38 sin(phi): a 1.8 mm gap
at the start, which closes as the frame comes square. So the start pose is one the real frame reaches by hand.

### No stop at square

At square both shoulder corners touch, but nothing holds them there: pushed further, each joint pivots on its other
shoulder corner and the frame racks the other way. The sim has no stop either (probe 3 below pushed it straight
through, from about +18.5 to -3.1 mm). The frame stays where it is left: released, it springs back by at most 0.06 deg
(0.43 mm of diagonal) in the sim. The robot has to stop itself; going past square is scored like falling short.

### Shear resistance (the reset check) and why one hand fails

At each reset the joint drag is checked with a 10 N push-pull gauge: hold the near rail down and push the far-left
stile 30 mm from its end, along the rails; the frame must start to rack at 1.5-2.5 N (fresh tenons if below; paste wax
on the tenon faces if above). In the sim each joint has Coulomb drag (mechkit's friction: the joint holds until the
torque on it exceeds 0.139 N*m; the loop-closing fourth joint is frictionless and the other three carry its share).
Measured in the sim with three force-capped "push gauges" (small blocks on sliders whose drive force was ramped
0.5 N/s), probe 3, no robot contact:

| Test | Result |
|---|---|
| near rail held, gauge on the far-left stile along -y (the reset check) | racks at 2.05 N |
| released at 0 N for 1.5 s | spring-back 0.055 deg (0.40 mm of diagonal) |
| one gauge at the near-right corner along the long diagonal (one hand) | the whole frame slides at 1.53 N, rack change -0.02 deg |
| two gauges squeezing the long diagonal (two hands, zero net force) | racks at 2.98 N each; the frame's centre moves 0.5 mm |
| kept squeezing | passes square, -3.07 mm, no stop |

One hand slides the frame at about half the force two hands need to rack it (1.53 vs 2.98 N). Real values depend on
the bench and the tenon fit; the reset check fixes the drag, and a second gauge check at each reset fixes the bench
against it: with nothing held, push the near-right outer corner along the long diagonal (toward the far-left corner);
the whole frame must slide, at about 1.5 N, without racking (diagonals unchanged within 1 mm; sim: slides at 1.53 N,
rack change 0.02 deg). If it racks instead, wipe the bench top clean and dry.

## The mechanism (what is physical, what is a rule)

- Four rigid members. Three joints are mechkit hinges (a tree: near rail, both stiles, far rail); the fourth, which
  closes the loop, is a plain revolute authored by the task file (mechkit builds trees only). Its closure error stayed
  under 0.005 mm in every run. Each joint is a pin at its shoulder; the real mortise's 3 mm of play also lets a rail
  slide along its stile, which the sim leaves out (the reset puts each rail's edge on its pencil mark).
- Joint drag is mechkit's elasto-plastic Coulomb model with a 0.05 deg stick travel (0.4 mm of diagonal).
- Frame material friction 0.2, averaged by PhysX with the oak station table's 0.5 (a plain collider at PhysX's default
  material): 0.35 on the table, about 0.7 against the pads.
- Nothing in the scoring or the scene moves the frame; the score is read from the members' poses.

## Scoring rules

d = |long diagonal - short diagonal|, outer corner to outer corner (a tape); start d = 20 mm.

- Latched 0 for the trial: any outer corner ever more than 25 mm from its start mark (slid or turned off its outline;
  squaring alone moves each corner 7.4 mm); any member's underside more than 10 mm off the table (lifted); real rig
  only: a rail pulled away from its stile (both shoulder corners more than 3 mm off it).
- 1.0 d <= 2 mm, each rail's edge within 1 mm of its pencil mark on both stiles (shoulder gaps closed <= 0.5 mm), the
  frame let go (no robot point within 2.5 cm of any member; robot points are each gripper's TCP, both pad tips and
  their midpoint) and still; 0.75 the same d and the same joint fit but still held or moving; 0.5 d <= 6 mm; 0.25
  d <= 12 mm; else 0. Success when 1.0 holds for 1 s; otherwise scored at the last step.
- The joint fit rules out a trapezoid whose diagonals happen to match (a rail slid along its mortise play). `score()`
  reads each rail end's two shoulder corners in the stile's frame: along the stile against their start (the pencil
  mark), across it for the gap. In the sim it is trivially true whenever d <= 2 mm: the joints are pins at the
  shoulders, so the rails cannot slide (the logged demo poses stay within 0.11 mm of the marks, the log's rounding),
  and the gap is 38 sin(rack), 1.8 mm at the start and at most 0.18 mm at d = 2 mm (0.11 mm at the demo's final
  +1.34 mm). The recorded demo's 1.0 is unchanged.

## Sim controls

`controls()` in the task file (run as `demo` in a debug copy of the final file, no frames): the right arm stays
parked; the left hand alone grips the far rail and drags it 40 mm toward the right (the arm's lag leaves about half of
that), then lifts it 25 mm. Dragged by one hand, the frame did not rack (diagonal difference +20.0 mm before, +20.1 mm
after); it slid and turned instead (centre 10 mm, 2.7 deg; a corner 21 mm off its mark). The lift then dragged it
further and raised it. Logged:

```
[sf] LATCH off-outline: a corner is 26.0 mm from its start mark (limit 25 mm) at control step 239, waypoints '[ctl] lift the far rail 25 mm' | 'parked'
[sf] LATCH lifted: a member's underside is 10.4 mm off the table (limit 10 mm) at control step 249, waypoints '[ctl] lift the far rail 25 mm' | 'parked'
task_score: 0
```

The 1.0 rung (square, let go, still) fires in the demo.

## How the demo does it

The shop fix, by hand: squeeze the long diagonal. Each gripper, fingers closed, starts 30 mm outside one end of the long
diagonal (the near-right and the far-left outer corner), finger tips 5 mm above the table, and pushes that corner along
the diagonal toward the other, so the two pushes cancel and the frame racks without sliding. The scripted demo reads
sim state, as the Guide allows: every control step the squeeze advances by a correction from the measured diagonal
difference (slower over the last 6 mm), and a balance term from the frame centre's drift along the diagonal shifts the
push from one hand to the other. When the difference is under 1 mm or changes sign, both hands stop pushing at once:
the arms lag their targets under load, so each target is set to where its hand actually is, 2 mm back (holding a
lagging target would keep pushing the frame past square). Then they back away 20 mm and lift clear.

In the debug run of the final demo (no frames): "[sf] demo: square to +0.88 mm at step 190 (within 1 mm): stop; hands
got +24.1 / +20.6 mm of +47.5 / +54.8 mm commanded" (the arms lag their targets by about 25-35 mm under the ~3 N
push); the frame settled at +1.34 mm, its centre within 1 mm of where it started, every corner within 6.3 mm of its
mark; task_score 1.0.

## Dream-1 run

One Dream-1 trial (1,200 steps, 60 s): task_score 0. Both arms moved about over the table beside the frame but never
pushed or gripped it; the diagonal difference stayed at 20 mm and no latch fired.

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 s at 20 Hz). The demo plan is 20 s; the recorded demo reached success and ended after 17.7 s.
- One fixed layout: `-E object_jitter=0` (the frame's members are joint-connected and never jittered). Dream-1 is hosted
  and not seeded; the demo is deterministic.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/square-racked-face-frame/square_racked_face_frame.py
```

It asserts the scoring rules on 15 hand-built frame poses (one with the far rail slid 2 mm off its pencil marks) and replays both arms' demo and control plans through the
kinematics at 20 Hz (every joint inside the URDF limits, IK error under 0.5 mm). Expected output (after the
`[sf] LATCH ...` lines that its off-outline and lifted cases print):

```
scoring: 15 cases pass; start diagonal difference +20.01 mm; members 0.129/0.103 kg; frame 0.466 kg
demo left: 20.0 s plan, worst IK 0.076 mm / 0.013 deg
demo right: 20.0 s plan, worst IK 0.092 mm / 0.011 deg
controls left: 18.0 s plan, worst IK 0.099 mm / 0.015 deg
controls right: 22.0 s plan, worst IK 0.054 mm / 0.015 deg
```

## Run and score

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/square-racked-face-frame/square_racked_face_frame.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This face frame is out of square. Make it square, with both diagonals the same length, without moving it off the blue corner marks or lifting it, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/square-racked-face-frame-dream
```

For the demo use `--policy yam_scripted` (same arguments, `--log-dir ~/yam-eval/square-racked-face-frame-demo`).
`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step. Videos:
`inspect-robots video <log>.json`.

## Reset between trials

Rack the frame back by hand to the start (long diagonal 20 +- 1 mm longer than the short one, each rail's edge on its
pencil mark), set its corners on the tape marks, and do both push-gauge checks above (the joint drag, then the
one-hand slide at the near-right corner). No sharp or heavy parts.

## Sources

- O*NET 47-2031.00 Carpenters, Core tasks: "Assemble and fasten materials to make frameworks or props, using hand tools
  and wood screws, nails, dowel pins, or glue." and "Verify trueness of structure, using plumb bob and level."
  https://www.onetonline.org/link/details/47-2031.00
- O*NET 51-7011.00 Cabinetmakers and Bench Carpenters, Core tasks: "Attach parts or subassemblies together to form
  completed units, using glue, dowels, nails, screws, or clamps." and "Verify dimensions or check the quality or fit of
  pieces to ensure adherence to specifications." https://www.onetonline.org/link/details/51-7011.00
- BLS Occupational Outlook Handbook: "Carpenters held about 889,700 jobs in 2025."
  https://www.bls.gov/ooh/construction-and-extraction/carpenters.htm ; "Cabinetmakers and bench carpenters 81,800"
  https://www.bls.gov/ooh/production/woodworkers.htm
- Squaring by the diagonals: "Check the carcase for square by measuring both diagonals. Adjust the carcase as needed
  until the measurements are equal." and "If the carcase is not square, apply clamp pressure across the larger
  diagonal." https://www.woodcraft.com/blog_entries/goof-proof-clamping ; "If the two diagonals match, the rectangular
  assembly is square, provided the opposite sides are equal and straight."
  https://www.woodsmith.com/article/jig-for-square-assemblies/
- Face-frame stock: "Most of the face frames we build use 3/4"-thick solid wood" / "Stiles: At least 1 1/2" wide"
  https://www.woodsmith.com/article/the-secrets-of-a-face-frame/
- Stock wall cabinets: "ex. W3036 = Wall Cabinet: 30-in wide x 36-in tall", W1812 listed among the 12 in high wall
  cabinets (a cabinet maker's 2018 wall-cabinet configuration sheet).
- Loose-tenon joiner: the maker's instruction manual: "the depth of the mortise should be ½ the length of the tenon"
  (depth settings 12, 15, 20, 25, 28 mm); it gives three mortise widths: snug (the tenon's width), the middle
  position 6 mm wider than the tenon, and the widest 10 mm wider. The maker's 5 x 30 mm beech tenon is 5 x 18.8 x 30
  mm, about 2 mm shorter than nominal to leave room for glue.
- Wood Handbook (FPL-GTR-190), Table 5-3a: yellow-poplar specific gravity 0.42 at 12 % moisture content.
- Engineering ToolBox friction coefficients: wood on wood, clean and dry, 0.25-0.5.
  https://www.engineeringtoolbox.com/friction-coefficients-d_778.html

## Closest existing tasks (checked against the live board and pull requests, 2026-11-14)

- "Square up a racked face frame by its diagonal" (another team's, not accepted; its 48 h hold lapses 2026-11-16
  05:50 UTC): the same idea. This task is an independent build that answers that review's points: a joint whose
  mortise play admits the start rack (with the clearance worked out), a start pose the real frame reaches, no reliance
  on a stop at square, and a drag that is set and checked at every reset.
- "Square up a racked gantry beam, then pull it in by both ends to its marks" (closed, not accepted): a guided machine
  beam, at reduced scale; not a frame glue-up.
- "Square up a leaning stack of cartons in place" (closed, not accepted): a loose stack, not a jointed frame.
- "Add snap-two-framing-struts-together-square" (closed, not accepted): snapping a corner joint together.
- "Roll a cabinet shelf wider than its face-frame opening through it..." (open): passes a shelf through a fixed opening;
  the frame is not adjusted.
- Known suites (BEHAVIOR-1K, LIBERO, RoboCasa): no task racks a jointed frame to square.

## Assets

All geometry is built from boxes in the task file; no third-party meshes, scans or scenes.
