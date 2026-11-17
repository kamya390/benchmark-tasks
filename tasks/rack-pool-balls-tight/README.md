# Collect fifteen scattered pool balls, rack them tight on the foot spot, then lift the triangle off clean

Instruction given to the policy: `Collect the balls, rack them tight on the foot spot and take the rack away.`

Files: `task.json`, `rack_pool_balls_tight.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`, a
sim control plan and an offline self-test), `mechkit.py` (unchanged helper: multi-box free part, per-episode memory),
`README.md`.

## Response to review

Review verdict (2026-11-16), verbatim:

> Task appears too simple as is. Balls should need to be collected before this placement step. Specify how the task
> can be implemented in a real-world evaluation reproducibly.

Each point:

- **Balls are now collected first.** The fifteen balls start scattered across the cloth at fifteen listed positions
  (all outside a chalked gather ring around the foot spot area), and the rack starts away from them, waiting beyond
  the foot spot. The robot has to pick every ball off the cloth, build the loose cluster at the foot spot, lower the
  triangle around it, and only then do the tightening and the lift that this task already scored. Scoring extends
  honestly over the new phase: n of 15 balls collected scores 0.25 n/15 until all are in, and the old ladder (0.25,
  0.5, 0.75, 1.0) now sits on top of full collection. Every tightness, orientation, head-ball and clearance
  threshold of the previous revision is unchanged.
- **Reproducible real-world setup.** The scatter is a fixed listed layout: fifteen (x, y) coordinates in cm in
  task.json's setup, each with a 0.5 cm tolerance, set with a printed grid or tape marks; the gather ring is a
  chalked circle of radius 16.5 cm centred 9 cm behind the spot, and collection is read from the final overhead
  photo (ball centre inside the ring). The products (balls, triangle, cloth), the measurement procedure for ball
  movement during the lift, the head-ball distance, both orientation angles (overhead grid photos) and the pair
  tightness (0.5 mm feeler gauge after the photos) were already specified and are kept under "Real objects and
  sources" and "Scoring rules".
- **Layout change this forced, disclosed.** With collection added, every cluster site must be reachable by a
  straight-down grasp, including the head ball's site, which is the farthest point the grippers must place a ball.
  At the previous foot spot (x 46 cm) that site is about 6 cm beyond the arms' reach, so the whole scene moved
  toward the robot: foot spot x 46 -> 34.5 cm, cloth centre x 43 -> 31.5 cm. For the same reason the rack now waits
  (and is finally set down) beyond the foot spot instead of behind the pack: behind, it would overhang the table's
  near edge. Nothing about the rack, pack, spot or scoring geometry changed relative to each other, and the
  tightening phase is the previously reviewed demo at the new spot.

## Environment

Isaac Lab-Arena 0.3.1 with yam-arena and inspect-robots, as the program's launchable installs them. Before every shell:
`source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/rack-pool-balls-tight/rack_pool_balls_tight.py
```

It asserts the scoring rules on 19 hand-built states: the scattered start (0); five balls collected (0.0833); the
cluster complete with the rack still waiting (0.25); a tight pack on the spot with the rack on it, lifted 100 mm
straight over it, carried away but still 60 mm up, set down 6 mm from the head ball, or at its waiting spot but
still sliding (all 0.25); a loose pack with the rack at its waiting spot (0.5); a tight pack with the rack at its
waiting spot (1.0), with the head ball 7 mm off, balls rolling or a pad touching a ball (0.75); the lift gate
latched (0); the back row 1 mm off; and four orientation cases (a correct pack turned 2 degrees passes; turned 180
or 10 degrees the pack's rear leaves the gather ring, so the score falls to collection credit; turned 4 degrees it
stays inside and the orientation check stops it at 0.5). It runs five lift-gate sequences, checks the orientation
angles directly, the scatter's spacing and clearances, that the whole cluster and pack lie inside the ring, that the
rack's waiting spot keeps 50 mm clearance even from a perfect pack, and replays both plans (the demo and control b)
through the kinematics at 20 Hz, every joint inside the URDF limits, IK error under 0.5 mm. Expected output:

```
   (back row 1 mm off, rack away: 22/30 pairs tight -> 0.5)
   (orientation: rotated 180 deg, head ball on the spot (apex faces the foot): 3/15 inside the ring -> 0.050)
   (orientation: rotated 10 deg, head ball on the spot: 14/15 inside the ring -> 0.233)
   (orientation: rotated 4 deg (apex inside 5, rear row outside 3): 15/15 inside the ring -> 0.500)
lift gate: 5 sequences pass
   (pack turned +0 deg about the head ball: apex 0.00 deg, rear row 0.00 deg, oriented True)
   (pack turned +180 deg about the head ball: apex 180.00 deg, rear row 0.00 deg, oriented False)
   (pack turned +10 deg about the head ball: apex 10.00 deg, rear row 10.00 deg, oriented False)
   (pack turned -4 deg about the head ball: apex 4.00 deg, rear row 4.00 deg, oriented False)
scatter: 15 balls, min spacing 87 mm, nearest 191 mm from the ring centre (ring r 165 + 20 mm clear zone)
cluster built: head ball 15.0 mm past the spot, widest gap 2.00 mm; rack lowering pose back-bar x 0.1147 (apex-touching 0.1112 + 3.5 mm forward)
rack at its waiting spot: 58.9 mm horizontally from the tight pack, lowest point 3.5 mm above the table, rack x 0.432..0.766
scoring: 19 cases pass
demo left: 165.7 s plan, worst IK 0.099 mm / 0.013 deg
demo right: 165.7 s plan, worst IK 0.099 mm / 0.017 deg
control b (not the demo) left: 156.8 s plan, worst IK 0.099 mm / 0.013 deg
control b (not the demo) right: 156.8 s plan, worst IK 0.099 mm / 0.017 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/rack-pool-balls-tight/rack_pool_balls_tight.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Collect the balls, rack them tight on the foot spot and take the rack away." \
  --scorer task_score --max-steps 6600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/rack-pool-balls-tight-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`), otherwise
`score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/rack-pool-balls-tight/rack_pool_balls_tight.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Collect the balls, rack them tight on the foot spot and take the rack away." \
  --scorer task_score --max-steps 6600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/rack-pool-balls-tight-demo
```

The demo is `demo()` in the task file, 165.7 s of plan in three phases:

1. **Collection (~96 s).** The left arm collects the balls scattered at y > 0, the right arm y < 0.
   Each ball is grasped around its equator from straight above (pads descend opened to 65 mm, 3.9 mm clear of the
   ball each side; every scattered ball stands at least 8 cm from its neighbours), carried at a height that clears
   standing balls, and set down on its site in the loose cluster (2 mm gaps, rows from the apex, centre-out, the
   head ball 15 mm past the spot). Only one hand is over the cluster at a time; the other fetches its next ball and
   waits at a staging hover on its own side. The grip yaw at each site keeps the pads over the gaps between placed
   neighbours (at least +8.9 mm pad clearance releasing 2 mm up); the three sites hemmed in by three neighbours take
   a 20 mm drop instead (+2.4 mm pad clearance; the balls land dead, restitution 0).
2. **The rack (~26 s).** Both hands pinch the waiting rack's back bar, carry it in an arc over the finished cluster
   (bar bottoms 35 mm above the ball tops) and lower it around the balls, 3.5 mm forward of the apex-touching pose,
   so its V walls stay about 1.8 mm clear of the head ball and its back bar about 4.4 mm clear of the rear row on
   the way down; then they release the bar.
3. **Tighten and lift (~44 s), as previously reviewed.** The same pinch in the cusps between back-row balls 1-2 and
   4-5, the 1 mm raise, the pull back (longer by the same 3.5 mm the rack was offset, so it ends at the same tuned
   end point; 3 mm/s, the last 3.5 mm at 1 mm/s), the 3 s settle, the 1.5 mm ease forward, and the slow lift. The
   rack is then carried forward, past the pack, back to its waiting spot and set down there (58.9 mm from the head
   ball even with the head dead on the spot), and the hands withdraw.

Recorded run: <<TBD-DEMO-RESULT>>

## Real objects and sources

- **Balls:** fifteen 2 1/4 in (57.15 mm, tolerance +-0.127 mm) balls of 5.5-6 oz (156-170 g), the WPA specification
  (Wikipedia, "Billiard ball", citing the WPA equipment specifications). Modelled at 57.15 mm and 165 g.
- **Rack:** a wooden 8-ball triangle for 2 1/4 in balls. Inside dimensions from a retailer's listing (pooldawg.com,
  "Wood 8 Ball Rack"): "10 5/8'' from head ball to rear center" and "11 5/8'' x 11 5/8'' x 11 5/8'' from rounded corner
  to rounded corner". Fifteen touching balls need 255.1 mm from the head ball's front to the rear row's back (4 x 57.15
  x sqrt(3)/2 + 57.15) and 285.75 mm corner to corner, so this rack leaves 14.8 mm of play along its axis. The model
  keeps the 269.9 mm head-to-rear figure exactly (side bars on the 60-degree lines of a tight pack, rear bar 14.8 mm
  behind its back row). Height 1.25 in (32 mm) and weight 0.5 lb (227 g) are from other retailers' listings of wooden
  8-ball triangles (13 x 11.5 x 1.3 in; 0.5 lb). The bar thickness (15 mm) is an estimate. A second source,
  dimensions.com ("Billiards Racks"), gives 11.25 x 10 in inner dimensions, which are the zero-play values; real racks
  are made with play so the balls can be pressed together.
- **Cloth, spot and ring:** a patch of worsted pool cloth on the station table stands for the foot end of the table's
  slate bed; the foot spot is a 1 in printed disc; the gather ring is a chalked circle of radius 16.5 cm (4 mm line)
  centred 9 cm behind the spot on the long axis. Cushions and pockets are far from the foot spot on a real table and
  are not modelled.

## Physics (what is physical, what is modelled)

- Balls are rigid spheres on the cloth; everything that happens to them comes from contacts with the rack, each other
  and the grippers. PhysX has no rolling resistance, so each ball carries angular damping 10 1/s, which slows a rolling
  ball as cloth does (a ball rolling at 5 mm/s stops within about 2 mm). Friction (PhysX average combine): ball-ball
  0.06, ball-cloth 0.20, ball-wood 0.30; restitution 0. These are estimates.
- Contact offset 0.1 mm and rest offset 0 on balls, rack and cloth, so a gap of 0.5 mm means something: a tight pack
  settles with every pair within 0.05 mm.
- The balls are collected with an equator grasp (the gripper opens to 94 mm); a dropped ball lands dead (restitution
  0) and the cloth's rolling resistance stops it within a couple of mm.
- Tightening is the real difficulty. Pushed through the rack's V, the balls jam: in six debug trials of demo variants
  of the previous revision the pull left the widest gap between 0.25 and 0.80 mm, and an arm moving in small jerks
  (about 1.3 mm) sets the pack coasting. A stationary tight pack needs the push to end gently.
- The rack is a free body of three wooden bars (227 g) that the grippers hold by friction. At its waiting spot its
  back bar lies on the 3 mm cloth and its apex on the bare table, a 0.5 degree tilt.

Probe (rack moved by a stiff driven slider, no robot; logs kept with the build): a tight pack settled with all 30 pairs
under 0.05 mm; lifted straight up, no ball moved (0.00 mm); lifted along an axis tilted 1 in 4 sideways (5 mm of drift
by 20 mm up), a ball moved 9.6 mm and gaps opened to 3.1 mm; a loose pack (2 mm gaps) lifted straight up kept its
2.0 mm gaps; a loose pack with the rack pulled back 10.9 mm at 3 mm/s closed to within 0.45 mm.

## Scoring rules

The table's long axis runs along x; the head of the table and its head string are at +x (away from the robot).

- Latched 0: once the rack's lowest point is 2 mm above the cloth, any ball's centre moves more than 3 mm horizontally
  before the rack has been set down away from the pack. If the rack comes back down flat on the cloth before that
  (lowest point within 0.5 mm, still), the reference is dropped and taken again at its next rise (`_gate_step`). The
  same rule covers the rack's carry-in: lowering it around the cluster must not shove any ball more than 3 mm, and
  once it stands on the cloth the gate re-arms for the final lift.
- Collection: a ball is collected once its centre is inside the gather ring. With n < 15 collected the score is
  0.25 n/15; collection is read from the final overhead photo against the chalk ring.
- 0.25 all fifteen collected; 0.5 that, the head (1) ball's centre within 10 mm of the spot, and the rack taken
  away: set down on the table (its lowest point within 5 mm of the table top, moving under 5 mm/s) with every part at
  least 50 mm horizontally, seen from above, from every ball (each bar's footprint against each ball's circle); 0.75
  that, at least 24 of the 30 touching pairs within 0.5 mm, and the pack facing the head of the table (below); 1.0
  that, all 30 within 0.5 mm, the head ball within 5 mm of the spot, all balls still (under 5 mm/s), and no gripper
  point (both TCPs, all four pad tips and pad middles) within 30 mm of any ball. Held 1 s. All thresholds unchanged
  from the previous revision.
- Real rig: two overhead photos against a grid on the cloth, one just before the rack starts to rise for the final
  lift and one after it has been set down; the trial fails (0) if any ball's centre moved more than 3 mm between
  them. The second photo also gives the collection count (centres inside the ring), the head ball's distance from
  the spot, the rack's horizontal clearance from the pack and both orientation angles. Only after both photos (the
  gauge can nudge a ball) are the 30 touching pairs checked with a 0.5 mm feeler gauge: a leaf held vertical, square
  to the line between the two centres, is lowered into the gap midway between them. Two spheres are closest at their
  equators on that line, so the pair is within 0.5 mm unless the leaf passes the equators and reaches the cloth. The
  leaf must not push either ball.
- Orientation (needed for 0.75 and 1.0). The table's long axis is x; the head of the table, where the apex points, is
  +x. (1) Apex: the line from the centroid of the 15 ball centres to the head ball within 5 degrees of the long axis,
  pointing toward the head string. (2) Rear row: the least-squares line through the five rear-row centres (balls 5, 13,
  15, 6, 12) within 3 degrees of square to the long axis. Why these limits: the centroid is 132 mm behind the head
  ball, so 5 degrees puts the rear row's centre at most 17 mm (under a third of a ball) off the long string, and the
  apex test is the one that rejects a pack facing the wrong way (turned 180 degrees, its rear row is still square); the
  rear row spans 229 mm, so 3 degrees puts its end balls at most 6.0 mm (a tenth of a ball) out of square, and since a
  tight pack is rigid this also holds its rotation to 3 degrees. A pack turned far enough also pushes its rear corners
  out of the gather ring, and the score falls to collection credit before the orientation check is reached. Real rig:
  both angles from the ball centres in the second overhead grid photo; with centres read to about 1 mm, the angles are
  good to about 0.6 degrees (apex) and 0.3 degrees (rear-row fit), well inside the limits.

## Sim controls

- **(a) Lift with drift** (`RK_DRIFT=0.005`: the hands drift 5 mm sideways over the first 20 mm of the final lift):
  <<TBD-CTL-A>>
- **(b) No tightening** (`controls_no_tighten()` with `RK_START_PAST=0`, so the cluster is built with the head ball
  on the spot): the whole demo, collection and rack placement included, with no pull back before the lift.
  <<TBD-CTL-B>>
- **(c) Partial collection** (`RK_ONLY_N=5`: the arms collect five balls and park): <<TBD-CTL-C>>

## How the demo does it

See "Run and score the full-score demo" above; the demo is deterministic (one fixed scatter, no jitter) and reads sim
state as the Guide allows: each hand's target is corrected every control step against its measured TCP in x and z
(and in y during the collection phase only, where lateral accuracy decides whether a ball lands on its site; during
the pull a y loop once pushed the rack sideways, so there it stays off).

## Dream-1 run

<<TBD-DREAM>>

## Run limit, runtime and seeds

- Run limit: 6600 steps (330 seconds at 20 Hz). The demo plan is 165.7 s: collection of
  fifteen balls is the bulk of it, and the limit leaves the same about-2x margin over the demo as the previous
  revision's 1,200 over its 41 s plan.
- One fixed layout: `-E object_jitter=0`. Dream-1 is hosted and not seeded. The demo is deterministic on one host
  (repeat runs on the same machine give the same numbers); results can differ slightly across GPU hosts.
- <<TBD-REPEATS>>
- Background, the variant sweep (previous revision, unchanged tightening): six debug trials of demo variants on the
  same host ended with widest gaps of 0.25-0.80 mm when the rack was set down; the shipped tightening is the variant
  that reached 0.25 mm.
- Runtime per trial: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up.

## Reset between trials

Set each ball with its centre on its listed scatter point (within 0.5 cm), using a printed grid or tape marks on the
cloth; every ball starts at least 2 cm outside the chalk ring and at least 8 cm from every other ball. Put the rack
at its waiting spot: apex away from the robot, back-bar centre 9.5 cm past the spot (x 44 cm), y 0, its back bar on
the cloth. Check that no ball touches another, none is inside the ring, and the wiped cloth is flat.

## Dup check (closest tasks, checked at packaging time)

Checked against the live repository and task board on 2026-11-14 (reconfirmed 2026-11-16):

- No open or closed pull request mentions pool balls, billiards, racking balls, a foot spot or a triangle rack. The
  nearest pull requests, all closed and not accepted: re-spotting snooker balls on their spots (#1579), stacking five
  balls into a pyramid in a square tray (#430), singulating one box from a tightly packed 3 x 3 cluster (#515), laying
  three floorboards tight together (#897).
- Accepted on the board: taking one bottle from a row of five without knocking any over. That task is about not
  toppling neighbours while removing one item; this one is about collecting scattered rolling balls into a
  close-packed cluster, closing its gaps inside a frame, and releasing the frame without moving any of them.
- External suites: BEHAVIOR-1K `clean_a_felt_pool_table_top` (cleaning only) and ManiSkill `roll_ball` (roll one
  ball to a goal). Neither collects and packs balls in a frame or lifts a frame off them.

## Assets

All geometry is built from boxes, a cylinder and spheres in the task file (no third-party meshes). `mechkit.py` is
this repository's helper.
