# Land a hoisted motor on its base's four studs: one hand runs the hoist, the other steers the load

Instruction given to the policy: `Set the motor down on its base with the hoist, all four feet over the base's studs and flat on the base, and leave the sling slack. Let go of the motor and the pendant when you are done.`

## Response to latest review (TK-CXPKK6, 2026-11-16)

> "The elastic sling addresses the physics suggestion, but the revised video shows only one stud through its
> foot. Please record the same full-score run with additional views that show all four feet seated on their
> studs, without restaging the motor."

- **All four feet are now shown, without restaging.** The revised demo is one 683-step full-score run: its full
  top-camera take is followed by four fixed observer-camera clips from that same run, one aimed at each
  stud/foot interface, and a simultaneous four-view final frame. Each close-up shows its distinct stud proud
  through the corresponding foot. At step 670 all four foot undersides were on the plate (reported z error
  0.0 mm), hole-to-stud errors were 0.2-0.3 mm, yaw error was 0.03 deg, the sling was 45.4 mm slack and the
  score was 1.0. The observer-camera option was enabled only for this scripted recording; it is absent from the
  normal three-camera Dream-1 run and does not change task physics, scripted actions or scoring.
- **The earlier elastic-sling feedback remains addressed.** The hook carries the eyebolt by an 85 cm
  endless polyester round sling (EN 1492-2, WLL 1 t, violet in the scene). Polyester round slings stretch about
  3% at WLL (EN 1492-2 verification elongation; maker data sheets quote <=3%), so the hitch is a spring-damper
  engaging at 85 cm: k = 1000 kg x g / (0.03 x 0.85 m) = 3.85e5 N/m, damping 570 N s/m (damping ratio 0.08
  against the 33 kg load, assumed - not from a data sheet), slack below 85 cm. Measured in sim: the hung motor
  stretches the sling 0.8 mm (= mg/k) and hangs still; the load still swings as a pendulum and the demo still
  has to land the feet on the studs under those dynamics. No scorer rule or threshold changed.

Files: `task.json`, `hoist_motor_onto_studs.py` (scene, the hoist's hold-to-run logic, `success()`, partial-credit
`score()` with its latched gates, the scripted `demo()` and an offline self-test), `mechkit.py` (multi-box parts,
sliders and per-step hooks added to the stock yam_arena runtime in memory), `README.md`.

## What the robot faces

A 33 kg NEMA 182T motor hangs from a hold-to-run electric chain hoist 6 cm above its base's four studs, 1.1 cm off the
stud pattern and turned 6 deg, because the hook is not parked exactly over the base. The hook carries the motor's
eyebolt by an 85 cm polyester round sling that stretches elastically under load (k 3.85e5 N/m from the sling's 3%
elongation at WLL; 0.8 mm static stretch under this motor) and goes slack once the load is set down. The hoist lowers
only while the pendant's DOWN button is held. Lowered alone, the motor lands on the stud tips out of line. One arm
has to hold DOWN while the other pulls the hanging motor about 1 cm against the pendulum's pull (about 3.6 N with
1.0 m from hook to centre of mass), turns it square, and keeps it there while it comes down, so that the 3 mm
chamfers can take the feet onto the studs (0.4 mm radial clearance). The robot cannot lift or drag the motor itself.
Neither hand may go under the motor before it is down on the plate, or between a foot and the base in the last 5 cm.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`.

### Real-trial equipment and affordable route

No custom lifting mechanism has to be bought or fabricated. Use an unpowered used or failed 3 hp NEMA 182T motor
(its electrical condition is irrelevant), or an inert 182T frame shell ballasted to 33 kg. Rent the lifting gear
for the trial: a published United Rentals schedule lists a 1,000 lb electric chain hoist at about $105 per day, and
Sunbelt Rentals lists its portable 2 ton gantry as CAT CLASS 044-0075 alongside electric-chain-hoist rentals.
Request a pendant-controlled unit with a low landing speed close to 1.2 m/min; any equivalent gear rated above
the 33 kg load is acceptable. This keeps the one-day equipment route to standard rental items rather than a
four-figure hoist/gantry purchase. Catalogs: [United Rentals hoists](https://www.unitedrentals.com/marketplace/equipment/tools-power-hand-surveying/hoist),
[Sunbelt gantry and hoist rentals](https://www.sunbeltrentals.com/equipment-rental/general-construction-tools/chain-hoists-air-winches/gantry-cranes/).

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/hoist-motor-onto-studs/hoist_motor_onto_studs.py
```

It prints the geometry (centre of mass, 1.00 m hook to centre of mass, 3.6 N pull at the seat) and asserts the geometry and the scoring rules on hand-built states (every level and every gate, including a gate
firing and then the motor being seated correctly, which still scores 0), and replays every demo plan through the
kinematics at 20 Hz (every joint inside the URDF limits; the demo's IK error under 0.5 mm). Expected output (the self-test also prints one
`LATCH` line per gate case to stderr):

```
scoring: 16 cases pass
demo           left: 33.3 s plan, worst IK 0.097 mm / 0.016 deg
demo           right: 36.9 s plan, worst IK 0.079 mm / 0.014 deg
down_only      right: 13.2 s plan, worst IK 0.078 mm / 0.013 deg
keep_pressing  left: 23.4 s plan, worst IK 0.097 mm / 0.016 deg
keep_pressing  right: 25.7 s plan, worst IK 0.075 mm / 0.013 deg
one_arm        right: 24.5 s plan, worst IK 0.082 mm / 0.015 deg
under          left: 14.0 s plan, worst IK 0.079 mm / 0.012 deg
under          right: 10.5 s plan, worst IK 0.078 mm / 0.013 deg
pinch          left: 28.1 s plan, worst IK 0.084 mm / 0.016 deg
pinch          right: 25.7 s plan, worst IK 0.075 mm / 0.013 deg
pinch_park     left: 20.0 s plan, worst IK 0.080 mm / 0.012 deg
pinch_park     right: 16.7 s plan, worst IK 0.078 mm / 0.013 deg
swing          left: 18.4 s plan, worst IK 0.083 mm / 0.013 deg
swing          right: 17.2 s plan, worst IK 0.078 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/hoist-motor-onto-studs/hoist_motor_onto_studs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the motor down on its base with the hoist, all four feet over the base's studs and flat on the base, and leave the sling slack. Let go of the motor and the pendant when you are done." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/hoist-motor-onto-studs-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`. The hoist logic needs `-E control_hz=20`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/hoist-motor-onto-studs/hoist_motor_onto_studs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the motor down on its base with the hoist, all four feet over the base's studs and flat on the base, and leave the sling slack. Let go of the motor and the pendant when you are done." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/hoist-motor-onto-studs-demo
```

The demo is `demo()` in the task file, about 34 s of plan including the camera passes; in sim it reaches success at
step 683 (34.2 s) including the 1 s success hold, inside the run limit of 1,200 steps (use `--max-steps 900`):

1. The left arm goes round the motor (not over it), grips the conduit box from above and carries the motor about
   1 cm over the studs, turning it the 6 deg square. Because the arm lags its target against the hanging motor's
   pull, the guided waypoints are corrected every control step from the motor's pose (a PI loop on the four holes'
   centre and the yaw, read from the simulator, as the program allows for scripted demos); the correction freezes
   2 mm above the stud tips.
2. The right arm holds DOWN on the pendant (button bottomed, 4 mm) for 6.9 s: the hoist lowers the motor 9.8 cm onto
   the base and runs 3-4 cm of sling slack; the left arm follows the motor down and the chamfers take the feet onto
   the studs.
3. The right arm lets go of DOWN and moves clear; the left arm opens, backs off and moves clear.
4. Each wrist camera in turn stands in front of the base while both arms finish moving clear (more than 5 cm from
   the motor). The motor sits flat on all four studs and the episode ends on success. Four fixed observer cameras
   record the four stud/foot interfaces for the review video; they are not enabled for Dream-1 and are not used by
   the scripted policy, entering that scripted run's observation only so the recorder captures them.

## Negative controls

Alternate plans, selected with the environment variable `HOIST_PLAN` (the shipped demo is `HOIST_PLAN` unset):

| `HOIST_PLAN` | What the robot does | Result in sim (seed 0) |
|---|---|---|
| `down_only` | Holds DOWN, no guiding | 0: the feet land on the stud tips, holes 10 mm / 5 mm off and turned 6 deg |
| `keep_pressing` | The demo, but DOWN is never let go | 0.75: seated and flat, but the button is still held and the chain runs out |
| `one_arm` | The right arm lines the motor up, lets go and then presses DOWN | 0: once let go the motor swings away (the arm knocks it turning as it leaves) and lands on the stud tips |
| `swing` | Shoves the hanging motor and lets it swing while it is lowered | 0: stud-strike gate fires (a foot hits a stud at 8.4 cm/s) |
| `idle` | Nothing (both arms hold their ready pose) | 0: the motor hangs still, chain taut |
| `under` | Fingers under a foot of the hanging motor | 0: under-load gate fires (a gripper point 2 mm below the hanging motor's feet, inside its outline, as the hand comes in low) |
| `pinch` | The demo until the studs are in the holes, then a fingertip on the plate under a foot while DOWN is pressed again | 0: the hand under the foot props the motor up; as it tips, a foot hits a stud at 5.3 cm/s and the stud-strike gate fires |


## Videos and screenshot

```bash
for log in ~/yam-eval/hoist-motor-onto-studs-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/hoist-motor-onto-studs-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

The submitted demo is one full-score recording with no re-staging: the top camera's complete 683-step take,
followed by final-state clips from four fixed observer cameras aimed at the four distinct stud/foot interfaces,
then all four views simultaneously. The four observer streams were captured at every control step of the same
run; only their brightness/gamma was graded in the edit to make shadowed interfaces legible, and labels were
added. There are no cuts within the top-camera action and no geometry or state edits. The recording pass renders
at 640 x 352; the submitted 1280 x 704 encode preserves each stream's full resolution in the four-view frame.
The scorer reads recorded poses, not pixels, and everything else matches the demo command above. The task file
also adds a fixed front fill light (render only, no physics).

For exact media reproduction, apply `recording_cameras.patch` to the clean `yam-isaac-lab-arena` checkout used by
the program and replace the scripted-demo command's camera options with
`-E overview_cam=True -E cam_width=640 -E cam_height=352`. The patch adds the four fixed observer streams with
their exact poses, focal length and clipping plane; it does not alter the task scene or physics. Render all streams
with `inspect-robots video <run.json>`.

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial: about 3 minutes of wall clock for the demo's 683 steps (flat on the base by step 330, then the
  camera passes) on a shared L40S (including Isaac Sim start-up); roughly 4 minutes for the full 1,200 steps.

## Reset between trials (real rig)

See `reset_notes` in task.json: limit switches and pendant E-stop on the hoist, an operator E-stop that cuts both
hoist and robot, abort on the under-load gate; reset with UP to 6.0 cm above the tips, the trolley to its parking stop
(plumb bob, within 2 mm), the motor to 6 +- 1 deg against a yaw stop, swing under 2 mm.

## Scoring

Evaluated every control step; levels are read at the last step except 0.25, which is latched.

On the real rig, synchronize fixed top, x-side and y-side cameras at 60 fps and calibrate image scale from the
known 190.5 x 114.3 mm stud pattern. For the 5 cm/s strike gate, measure a contacting foot's displacement over
the preceding 0.10 s; the same views show the under-load and pinch regions. Use a feeler gauge for the 1 mm flat
check and a tape for sling slack.

- 0 (whole trial, latched) if any part of the robot was below the motor's lowest point inside its plan outline grown
  by 1 cm before all four feet were down on the plate, or between a foot and the plate while that foot was under
  5 cm above it, or a foot touched a stud while moving faster than 5 cm/s.
- 1.0 (success, held 1 s): all four studs through their holes, every foot underside within 1 mm of the plate, the
  chain 1 to 15 cm slack, both grippers at least 5 cm from the motor, both pendant buttons up, the motor still.
- 0.75: all four through and flat, but not all of the rest.
- 0.5: the load off the hoist (chain at least 5 mm slack) with two or more studs through.
- 0.25: at some point all four holes were within 3 mm of their studs seen from above with the feet under 1 cm above
  the stud tips.
- Otherwise 0.
