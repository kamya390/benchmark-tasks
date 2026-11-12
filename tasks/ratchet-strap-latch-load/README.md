# Ratchet a tie-down strap taut round a crate and latch its handle before the deck is jolted

Instruction given to the policy: `Strap the crate down to the deck.`

Files: `task.json`, `ratchet_strap_latch_load.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/ratchet-strap-latch-load/ratchet_strap_latch_load.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 18 cases pass (demo winds 4 teeth: 60 N in the strap)
left: 34.1 s plan, worst IK 0.090 mm / 0.013 deg
right: 34.1 s plan, worst IK 0.078 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/ratchet-strap-latch-load/ratchet_strap_latch_load.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Strap the crate down to the deck." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/ratchet-strap-latch-load-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/ratchet-strap-latch-load/ratchet_strap_latch_load.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Strap the crate down to the deck." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/ratchet-strap-latch-load-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 34 s of plan (about 882 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/ratchet-strap-latch-load-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/ratchet-strap-latch-load-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## The rig (from the task file)

Positions in cm from the midpoint between the arm bases (x forward, y left). Heights are above the deck's top.

| Part | Where and what | In `ratchet_strap_latch_load.py` |
|---|---|---|
| Deck section | Steel chequer plate 24 x 24 x 1 cm, centred at x 29, y -11, top 1.2 cm above the table. With its ballast and carriage it moves 20 kg (the plate alone is about 4.5 kg). | `DECK_O`, `DECK_KG` |
| Slide | Along x, 15 cm of travel from the rear stop (the start) to the end stop. | `RUN`, `kit.slider("deck_run", ...)` |
| Drive | Holds the deck on its rear stop with 10 N, then pulls it forward with 10 N: about 0.47 m/s^2, reaching the stop at 0.38 m/s. | `K_DECK`, `FAR` |
| Stake board | Plywood, 1.2 x 10.6 x 12 cm, centred at x 24.9, y -7.5. | `STAKE_X` |
| Anchor stake | Steel, 1.2 x 1 x 11.5 cm, at x 26.2, y -1.0. | deck part |
| Ratchet | A lever ratchet for 3 cm webbing, mounted on its side on a 2.6 x 1.6 x 8 cm steel post at x 26.6, y -14.5, so the mandrel and handle pin are vertical. The pin is at x 26.2, y -14.5, 9 cm up. | `HX`, `HY`, `BAND_Z` |
| Handle | A 7.2 cm lever. The grip post (1.2 x 1.2 x 3.6 cm) is 6.4 cm from the pin. It starts 40 deg open; 0 is shut along -x, and it opens toward -y. | `R_POST`, `START_DEG` |
| Strap | 3 cm webbing on a 9 cm-high centre line, 23.3 cm from anchor to mandrel: a 5.95 cm leg at y -1.95, 11.4 cm across the crate's far face (12 mm off it, band centre at x 32.4), and a 5.95 cm leg at y -13.05. | `GAP`, `LEG_OFF`, `LEG_END_X` |
| Crate | Pine, ballasted to 1 kg, 5.5 x 10 x 14 cm, centred at x 28.3, y -7.5, against the stake board. | `CRATE`, `CX`, `CY` |

## Reset between trials

Engage the deck's catch first, then:
1. Pull the ratchet's release and pay the strap out to its slack mark: 12 mm off the crate in sim, or the mark set in
   calibration step 3 on a real rig.
2. Set the handle 40 deg open.
3. Stand the crate against the stake board.
4. Run the deck back by hand onto its rear stop, raising the drive weight, until the catch holds it.

## Real-rig build

- **Slide.** A profile linear guide (one rail, two carriages) under a base plate that the ballast and the chequer
  plate are bolted to. Hard rubber blocks set the rear stop at the start position.
- **Actuator.** A weight of about 1 kg on a cord over a pulley in front of the stop. This gives about 10 N, as in
  sim. It cannot push harder than its own weight, and with 15 cm of run it cannot go faster than about 0.4 m/s.
  The weight's drop is shorter than the cord, so it lands on a pad before the cord goes slack.
- **Catch.** A spring latch holds the deck on its rear stop. It is released by a solenoid that must be powered to
  release, so a power cut or the e-stop leaves the deck held. It plays the part of the sim's trip: the operator, or
  the robot controller's timer, releases it once both grippers have been 3 cm clear and still for 1 s.
- **End stop.** A steel block bolted to the table, faced with a 3 mm hard polyurethane pad. The sim's stop is
  rigid; the pad only stops metal-on-metal ringing.
- **Load cell.** The anchor stake is mounted on an S-type load cell (0-200 N) aligned with the strap's left leg, read
  on a display beside the rig.

## Real-rig calibration

In sim the tension is a model: every full stroke winds exactly 5 mm, and every mm past the crate adds 8 N. That
8 N/mm is a lumped stiffness for the webbing, its bedding on the crate's edges and the mandrel's slack, not a
property of the webbing alone. On a real ratchet the strap wound per stroke changes as the wrap on the mandrel
grows, so the rig does not count strokes: it reads the tension.

1. **Load cell, once and after any knock.** Off the rig, hang known masses from the cell: 1.02, 2.04, 5.10 and
   10.20 kg (10, 20, 50 and 100 N). The readings must be within 1 N. At the start of each session, zero it on the rig
   with the strap slack, then check it with the 5.10 kg mass on a cord over a pulley in line with the leg: 50 +/- 1 N.
2. **Taut means at least 50 N on the anchor leg.** Friction where the strap turns round the crate's corners makes
   the anchor leg the less loaded of the two legs. So 50 N there means at least 50 N in the ratchet leg too.
3. **Starting slack.** From fully slack, give full strokes and read the cell after each one. Set the slack mark
   (where the webbing enters the mandrel slot) so 3 strokes read under 50 N and 4 read 50 N or more, as in sim
   (about 20 N and 60 N). Repeat 3 times.
4. **Handle angles.** Fix a protractor plate round the pin. Mark the buckle's real angles: where a stroke clicks a
   tooth, where the handle locks shut, and where it releases the strap. The scorer's 1.5 deg shut and its release
   position are read against these marks.
5. **The jolt.** Release the catch with the crate and strap removed. Time the deck over the last 5 cm before the stop
   (two marks, video at 60 fps or more). It must arrive at 0.38 +/- 0.04 m/s. Add or remove weight until it does.

## Guarding and safe operation of the moving deck

The deck stores at most about 1.5 J (21 kg at 0.38 m/s). The hazards are the pinch between the deck and the end
stop, the pinch at the rear stop, and the falling weight.

- **Fence.** A fixed polycarbonate guard round the deck's sides and front, as high as the deck's top plus 10 cm,
  covers both stops and the pulley. The weight runs inside a closed tube. The top stays open for the robot.
- **E-stop.** One e-stop cuts the robot's power and the catch solenoid together. With no power the catch stays
  engaged, so the deck cannot be released.
- **Limited force and speed.** The gravity drive is limited to its weight (about 10 N) and its 15 cm run. There is
  no motor to drive the deck harder.
- **Operation.**
  - Release the catch only with the robot parked and no one's hands inside the fence.
  - After the trip, wait for the deck to come to rest on the stop.
  - Engage the catch before anyone reaches in to reset the strap or the crate.
  - Run the deck back by hand only.
