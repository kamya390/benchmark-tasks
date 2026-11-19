# Collect fifteen scattered pool balls, rack them tight on the foot spot, then lift the triangle off clean

Instruction given to the policy: `Collect the balls, rack them tight on the foot spot and take the rack away.`

Files: `task.json`, `rack_pool_balls_tight.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`, a
sim control plan and an offline self-test), `mechkit.py` (unchanged helper: multi-box free part, per-episode memory),
`README.md`.

## Response to review

Review verdict (2026-11-18), verbatim:

> Scoring (not met): The specified feeler-gauge check occurs after removal and cannot establish all 30 submillimetre
> gaps at the earlier rack rise.
>
> What to change: A new scoring issue, not raised in the earlier review: the feeler-gauge check after rack removal
> does not establish whether the 30 gaps met the 0.5 mm threshold at the earlier rise. Specify a practical,
> non-disturbing real-trial check at that moment, or replace the rise-time threshold with a measurable condition that
> still prevents tightening the pack after removal.

We took the second option, in the scorer and its tests:

- **The rise-time gap threshold is gone.** Tightness (24 or 30 of the 30 neighbour gaps within 0.5 mm), head
  distance and orientation are now read once, on the final state, which is exactly what the after-trial feeler-gauge
  check and the final overhead frame measure on a real table.
- **Hands off after the rise replaces it.** From the valid 2 mm rise to the end of the trial, no gripper point (both
  TCPs, all four pad tips and pad middles) may come within 10 mm of any ball surface; if one does, the score is
  capped at 0.25 (collection credit). Closing a gap needs contact, so once the rack has risen nothing can tighten the
  pack: the rack is in the air or set down 50 mm clear, and the hands are kept off. A full retry (the rack set back
  flat around the pack) starts a new removal, because touching balls while racking is allowed. `_hands_step` does
  this every control step; five whole sequences test it (touch before the rise allowed; the demo's 12 mm lift
  clearance passes; a poke while the rack is in the air, and a nudge after a clear set-down, both cap the score; a
  touch followed by a full retry passes).
- **The rack stays away after its clear set-down.** Once set down 50 mm clear, every rack part must stay at least
  50 mm (horizontally) from every ball to the end of the trial, or the score is capped at 0.25. Without this, the
  rack could be picked up again and its bars used to push a loose pack tight after the scored lift-off.
  `_rack_back_step` checks it every step; three sequences test it (the demo's rack left 61 mm away passes; a rack
  picked up again and pushed against the pack caps the score; the rack passing over the pack during carry-out,
  before its set-down, is allowed). On the real rig this is read from the overhead video at centimetre scale.
- **Packing by hand before the rack goes down is allowed.** The scored skill is the enclosure, the lift-off without
  moving any ball over 3 mm, and the hands-off and rack-away rules after the rise; how the balls got tight while the
  rack was down is not prescribed.
- **Measurable on the real rig without disturbing anything.** On a real table the rule is simply that no part of
  either gripper touches a ball from the rise to the end of the trial, judged from the overhead video and a low side
  video (a touch is visible, and it moves the ball). The sim cannot sense every finger surface, so it applies the
  same rule conservatively as a 10 mm standoff of the sampled pad and TCP points. The feeler gauge is used only after
  the trial has ended.
- **The submitted demo is unchanged and still scores 1.0** (re-scored offline below). Every other threshold is
  unchanged.

Review verdict (2026-11-17), verbatim:

> Construct validity (not met): The rack starts far enough from a completed pack to satisfy the clearance condition,
> so a robot can earn full credit by arranging the balls without using it.
>
> Scoring (not met): The rear-row angle uses five specified ball identities, excluding valid tight racks with a
> different rear-row arrangement that the instruction does not prohibit.
>
> What to change: require the rack to surround the pack and be lifted away for full credit; measure the rear-row
> orientation from the actual rear row rather than requiring five particular balls; accept valid tight arrangements;
> add tests for an untouched rack and alternative valid arrangements.

Each new point is addressed in the scorer, not just in this description:

- **Use of the rack is an ordered episode condition.** `_gate_step` records all fifteen ball outlines inside the
  three inner rack faces in the rack's last down, still state and continuously through the 2 mm rise crossing. Losing
  that enclosure at any pre-rise height invalidates the observation; only setting the validly lifted rack 50 mm clear
  completes removal. A perfect final pack next to the untouched waiting rack remains at 0.25, as does enclosing loose
  balls once, sliding that rack away, hand-forming a tight pack and later lifting the empty rack. (Tightening after
  the rack has been removed is now ruled out by the hands-off rule above, which replaced the rise-time tightness
  check.) The old 3 mm no-disturbance latch runs from valid rise through set-down.
- **Contacts and rear row come from geometry.** `pair_gaps` sorts all pairwise centre distances and checks the thirty
  physical neighbours; `orientation` selects the five centres farthest behind the head ball along the pack's own
  axis. Neither depends on non-head ball identities. A test swaps four pairs of non-head balls across rows and still
  scores 1.0 after a valid removal.
- **Real evaluation shows the sequence.** Continuous calibrated overhead video retains the frame with all ball
  outlines inside the rack, the rack passing above the pack, and the final clear set-down. The same view supports the
  before/after 3 mm check, the hands-off distance, and identifies the physical thirty neighbour gaps for the
  after-trial feeler-gauge check.

Earlier review verdict (2026-11-16), verbatim:

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

It asserts the scoring rules on 24 hand-built states: the scattered start (0); five balls collected (0.0833); the
cluster complete with the rack still waiting (0.25); a tight pack on the spot with the rack on it, lifted 100 mm
straight over it, carried away but still 60 mm up, set down 6 mm from the head ball, or at its waiting spot but
still sliding (all 0.25); a perfect pack beside the untouched rack (0.25); a loose pack after a valid rack removal
(0.5); a tight pack after valid removal (1.0), including after non-head balls are permuted across rows and when the
pack was loose at the rise but tightened by the rack before it rose clear with hands kept off; a tight pack after a
hand came within 10 mm of a ball after the rise, or after the rack was brought back to the pack following its clear
set-down (both 0.25); with the head
ball 7 mm off, balls rolling or a pad touching a ball (0.75); the lift gate
latched (0); the back row 1 mm off; and four orientation cases (a correct pack turned 2 degrees passes; turned 180
or 10 degrees the pack's rear leaves the gather ring, so the score falls to collection credit; turned 4 degrees it
stays inside and the orientation check stops it at 0.5). It runs nine whole rack-use/lift-gate sequences, five
hands-off sequences and three rack-stays-away sequences, checks the orientation
angles directly, the scatter's spacing and clearances, that the whole cluster and pack lie inside the ring, that the
rack's waiting spot keeps 50 mm clearance even from a perfect pack, and replays both plans (the demo and control b)
through the kinematics at 20 Hz, every joint inside the URDF limits, IK error under 0.5 mm. Expected output:

```
   (back row 1 mm off, rack away: 22/30 pairs tight -> 0.5)
   (orientation: rotated 180 deg, head ball on the spot (apex faces the foot): 3/15 inside the ring -> 0.050)
   (orientation: rotated 10 deg, head ball on the spot: 14/15 inside the ring -> 0.233)
   (orientation: rotated 4 deg (apex inside 5, rear row outside 3): 15/15 inside the ring -> 0.500)
lift gate: 9 sequences pass
hands off: 5 sequences pass
rack stays away: 3 sequences pass
   (pack turned +0 deg about the head ball: apex 0.00 deg, rear row 0.00 deg, oriented True)
   (pack turned +180 deg about the head ball: apex 180.00 deg, rear row 0.00 deg, oriented False)
   (pack turned +10 deg about the head ball: apex 10.00 deg, rear row 10.00 deg, oriented False)
   (pack turned -4 deg about the head ball: apex 4.00 deg, rear row 4.00 deg, oriented False)
scatter: 15 balls, min spacing 87 mm, nearest 191 mm from the ring centre (ring r 165 + 20 mm clear zone)
cluster built: head ball 15.0 mm past the spot, widest gap 2.00 mm; rack lowering pose back-bar x 0.1147 (apex-touching 0.1112 + 3.5 mm forward)
rack at its waiting spot: 58.9 mm horizontally from the tight pack, lowest point 3.5 mm above the table, rack x 0.432..0.766
scoring: 24 cases pass
demo left: 180.7 s plan, worst IK 0.099 mm / 0.018 deg
demo right: 180.7 s plan, worst IK 0.099 mm / 0.018 deg
control b (not the demo) left: 171.8 s plan, worst IK 0.099 mm / 0.018 deg
control b (not the demo) right: 171.8 s plan, worst IK 0.099 mm / 0.018 deg
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

The demo is `demo()` in the task file, 180.7 s of plan in three phases:

1. **Collection (~96 s).** The left arm collects the balls scattered at y > 0, the right arm y < 0.
   Each ball is grasped around its equator from straight above (pads descend opened to 65 mm, 3.9 mm clear of the
   ball each side; every scattered ball stands at least 8 cm from its neighbours), carried at a height that clears
   standing balls, and set down on its site in the loose cluster (2 mm gaps, rows from the apex, centre-out, the
   head ball 15 mm past the spot). Only one hand is over the cluster at a time; the other fetches its next ball and
   waits at a staging hover on its own side. The grip yaw at each site keeps the pads over the gaps between placed
   neighbours (at least +8.9 mm pad clearance releasing 2 mm up); the three sites hemmed in by three neighbours take
   a 20 mm drop instead (+2.4 mm pad clearance; the balls land dead, restitution 0).
2. **The rack (~41 s, including a 15 s pre-pickup settle).** Both hands wait for the last placed ball to settle,
   then pinch the waiting rack's back bar and carry it in an arc over the finished cluster
   (bar bottoms 35 mm above the ball tops) and lower it around the balls, 3.5 mm forward of the apex-touching pose,
   so its V walls stay about 1.8 mm clear of the head ball and its back bar about 4.4 mm clear of the rear row on
   the way down; then they release the bar.
3. **Tighten and lift (~44 s), as previously reviewed.** The same pinch in the cusps between back-row balls 1-2 and
   4-5, the 1 mm raise, the pull back (longer by the same 3.5 mm the rack was offset, so it ends at the same tuned
   end point; 3 mm/s, the last 3.5 mm at 1 mm/s), the 3 s settle, the 1.5 mm ease forward, and the slow lift. The
   rack is then carried forward, past the pack, back to its waiting spot and set down there (58.9 mm from the head
   ball even with the head dead on the spot), and the hands withdraw.

Recorded run (scored live by the previous revision's scorer): it first observed all fifteen balls enclosed at
control step 2,614; its last valid down observation was at 3,032 and enclosure remained true through the 2 mm rise
at 3,243. It observed the rack set down 61 mm from the nearest ball at 3,528. At rise and set-down all 30 physical
neighbour gaps were within 0.5 mm (widest 0.39 mm), the head ball was 4.4 mm from the spot, maximum ball motion
during removal was 0.00 mm, the apex and actual rear row were each 0.01 degrees from target, and `task_score` was 1.0.

Offline re-score under this revision's scorer: 1.0. The final-state terms are the logged values above. For the new
hands-off rule, a debug run with `RK_DEBUG=1` sent bit-identical joint commands at all 3,555 steps and hit the same
events (rise 3,243, set-down 3,528, same gaps); it logged the nearest gripper point to any ball every 10 steps: 11.9 mm
at step 3,240 just before the rise (the pads hold the back bar, 14.8 mm behind the rear row), 13.0 mm at 3,250, and
rising from there to over 100 mm after the set-down. So no gripper point came within the 10 mm limit after the rise.
After its set-down the rack stayed 61-62 mm from the nearest ball to the end (the 50 mm rack-away rule).
The live scorer also prints this minimum when the rack is set down.

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

- **Rack-use sequence and latched 0.** In the rack's last down, still state before the scored rise, all fifteen ball
  outlines must be inside its three inner faces, and that projected enclosure must remain true through the 2 mm rise
  crossing. Losing it at any pre-rise height erases the observation. Once the valid rack clears 2 mm, any ball centre
  moving more than 3 mm
  horizontally before it is set down away latches 0. Only returning it flat around the pack takes a new reference on
  a retry; touching empty cloth during carry-out keeps the same reference until clear set-down. Moving the empty rack
  before enclosure neither satisfies nor fails this sequence.
- Collection: a ball is collected once its centre is inside the gather ring. With n < 15 collected the score is
  0.25 n/15; collection is read from the final overhead photo against the chalk ring.
- 0.25 all fifteen collected; 0.5 that, the valid rack-use sequence completed, the head (1) ball's centre within
  10 mm of the spot, the hands-off rule kept, and the rack set down on the table (lowest point within 5 mm, under
  5 mm/s) with every part at least 50 mm horizontally from every ball; 0.75 that, finally at least 24 of the 30
  physically closest pairs are within 0.5 mm and the pack faces the head of the table; 1.0 that, finally all 30 are
  within 0.5 mm, the head is within 5 mm, all balls are still (under 5 mm/s) and no gripper point (both TCPs, all
  four pad tips and pad middles) is within 30 mm of any ball. Held 1 s. Every earlier threshold is unchanged; the
  rise-time copies of the tightness, head and orientation checks were removed.
- **Hands off after the rise.** From the valid 2 mm rise to the end of the trial, any gripper touching a ball caps
  the score at 0.25; the sim tests this as any sampled gripper point (TCPs, pad tips, pad middles) within 10 mm of
  any ball surface. Packing needs contact, so this is what prevents tightening the pack after
  the rack has been removed. A full retry (rack back flat around the pack) starts a new removal and clears it.
- **Rack stays away.** After its clear set-down, any rack part coming within 50 mm (horizontally) of any ball, before
  the trial ends, caps the score at 0.25.
- Real rig: continuous calibrated overhead video shows the rack enclosing every ball before the final rise, passing
  above the pack, and set down clear. Compare the frame just before rise with the frames up to set-down; the trial
  fails (0) if any ball centre moved more than 3 mm. Hands off: the overhead video and a low side video show whether
  any gripper part touches a ball from the rise to the end; any touch caps the score at 0.25 (the sim's 10 mm
  standoff of sampled pad points is its conservative stand-in for touch). The final frame gives collection count, the head
  ball's distance from the spot, the rack's horizontal clearance from the pack and both orientation angles. Only
  after the trial has ended and the final photo is taken (the gauge can nudge a ball) are the 30 touching pairs
  checked with a 0.5 mm feeler gauge; because the hands were kept off since the rise, these final gaps are the gaps
  the rack left: a leaf held vertical, square
  to the line between the two centres, is lowered into the gap midway between them. Two spheres are closest at their
  equators on that line, so the pair is within 0.5 mm unless the leaf passes the equators and reaches the cloth. The
  leaf must not push either ball.
- Orientation (needed for 0.75 and 1.0). The table's long axis is x; the head of the table, where the apex points, is
  +x. (1) Apex: the line from the centroid of the 15 ball centres to the head ball within 5 degrees of the long axis,
  pointing toward the head string. (2) Rear row: the least-squares line through the actual five centres farthest
  behind the head ball along the pack's own axis, within 3 degrees of square to the long axis. Why these limits: the
  centroid is 132 mm behind the head
  ball, so 5 degrees puts the rear row's centre at most 17 mm (under a third of a ball) off the long string, and the
  apex test is the one that rejects a pack facing the wrong way (turned 180 degrees, its rear row is still square); the
  rear row spans 229 mm, so 3 degrees puts its end balls at most 6.0 mm (a tenth of a ball) out of square, and since a
  tight pack is rigid this also holds its rotation to 3 degrees. A pack turned far enough also pushes its rear corners
  out of the gather ring, and the score falls to collection credit before the orientation check is reached. Real rig:
  both angles from the ball centres in the second overhead grid photo; with centres read to about 1 mm, the angles are
  good to about 0.6 degrees (apex) and 0.3 degrees (rear-row fit), well inside the limits.

## Scorer controls

- **(a) Lift with drift** (`RK_DRIFT=0.005`: the hands drift 5 mm sideways over the first 20 mm of the final lift):
  the 4 mm whole-sequence regression latches 0; an earlier GPU run of the same physical gate naturally moved one
  ball 3.66 mm after rise and scored 0. The submitted thresholds and gate logic are unchanged.
- **(b) No tightening** (`controls_no_tighten()` with `RK_START_PAST=0`, so the cluster is built with the head ball
  on the spot): the whole demo, collection and rack placement included, with no pull back before the lift.
  Its 171.8 s plan passes IK; the offline whole-episode final state has the valid removal but loose gaps and scores
  0.5. This is a scorer regression, not represented as a second submitted video.
- **(c) Partial collection** (`RK_ONLY_N=8`): GPU probe p1 collected eight balls, stopped without moving the rack,
  and scored exactly `0.25 * 8/15 = 0.1333`.

## How the demo does it

See "Run and score the full-score demo" above; the demo is deterministic (one fixed scatter, no jitter) and reads sim
state as the Guide allows: each hand's target is corrected every control step against its measured TCP in x and z
(and in y during the collection phase only, where lateral accuracy decides whether a ball lands on its site; during
the pull a y loop once pushed the rack sideways, so there it stays off).

## Dream-1 run

Dream-1 ran the full 6,600-step limit on the submitted collection layout, did not collect a ball, and scored 0.0.
The instruction and physical scene are unchanged in this scorer-only revision; the scoring changes only affect the
tiers above full collection, so they cannot change that result.

## Run limit, runtime and seeds

- Run limit: 6600 steps (330 seconds at 20 Hz). The demo plan is 180.7 s: collection of
  fifteen balls is the bulk of it, and the limit leaves the same about-2x margin over the demo as the previous
  revision's 1,200 over its 41 s plan.
- One fixed layout: `-E object_jitter=0`. Dream-1 is hosted and not seeded. The demo is deterministic on one host
  (repeat runs on the same machine give the same numbers); results can differ slightly across GPU hosts.
- Recorded run: 1/1 full recording run scored 1.0 live under the previous revision's scorer; it stored all 3,556
  frames and is the submitted demo. Re-scored offline under this revision's scorer: 1.0 (see "Recorded run").
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
