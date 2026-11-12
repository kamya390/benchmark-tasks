# Free a bottle jammed in a capper's infeed starwheel

Instruction given to the policy: `The capper's infeed has jammed on a bottle and the line is stopped and locked out. Take the jammed bottle out of the starwheel and put it in the red reject bin.`

Files: `task.json`, `starwheel_jam_back_off.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/starwheel-jam-back-off/starwheel_jam_back_off.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 18 cases pass
right: 29.7 s plan, worst IK 0.094 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/starwheel-jam-back-off/starwheel_jam_back_off.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The capper's infeed has jammed on a bottle and the line is stopped and locked out. Take the jammed bottle out of the starwheel and put it in the red reject bin." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/starwheel-jam-back-off-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/starwheel-jam-back-off/starwheel_jam_back_off.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The capper's infeed has jammed on a bottle and the line is stopped and locked out. Take the jammed bottle out of the starwheel and put it in the red reject bin." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/starwheel-jam-back-off-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 30 s of plan (about 794 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/starwheel-jam-back-off-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/starwheel-jam-back-off-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Starting layout (from the task file)

Positions in cm from the midpoint between the arm bases (x forward, y left). Angles are yaw about the vertical,
anticlockwise from +x seen from above. The wheel's angle is 0 at the start; turning back (against the flow) is
anticlockwise.

- **Starwheel, indexed (0 deg):** the shaft is at x 30, y -13. The lobes point at -90, 0, 90 and 180 deg (along -y, +x,
  +y and -x). A white index mark (0.4 x 3 cm) on the -90 deg lobe, centred 5.5 cm from the shaft, reads against a
  degree scale on the deck.
- **Guide segments:** each is 3.8 cm long (tangential), 0.8 cm thick (radial) and 5 cm tall, and stands on the deck.
  Each is square to its radius from the shaft, so its yaw is that radius's angle.

  | Segment | Centre x, y (cm) | Yaw (deg) | Inner face from shaft (cm) |
  |---|---|---|---|
  | 1 | 38.56, -10.22 | 18 | 8.60 |
  | 2 | 37.89, -6.61 | 39 | 9.75 |
  | 3 | 34.98, -4.38 | 60 | 9.55 |
  | 4 | 31.64, -2.63 | 81 | 10.10 |
  | 5 | 27.70, -2.19 | 102 | 10.65 |
  | 6 | 23.68, -3.27 | 123 | 11.20 |

- **Bottle:** upright, cap up, base centre at x 33.70, y -6.59 (7.4 cm from the shaft at 60 deg), yaw 60 deg, so its
  faces are square to that radius. Its outer face is 1.5 mm off segment 3, and its nearest corner is 1.7 mm off the
  90 deg (trailing) lobe.

These are `GUIDE`, `LOBES_DEG`, `POCKET_DEG`/`POCKET_R` and `_guide()` in `starwheel_jam_back_off.py`.

## Reset between trials

The deck, guide, chevrons, shaft, isolator and bin never move: they are bolted or doweled at the setup positions.

**Sim:** the wheel returns to 0 deg and the pinch is re-made.

**Real rig.** The sim's pinch is a hold on the bottle that the wheel's angle releases, not an elastic wedge. A squeeze
through the lobe would itself back-drive a 0.06 N*m wheel, so the rig builds the pinch as a switched holding magnet:

- a DC holding electromagnet, set flush in the deck's top under the bottle's base centre;
- a 25 mm x 1 mm mild-steel disc, bonded under the bottle's base;
- a latching relay feeding the coil. A cam flag on the wheel trips it at 6.0 to 6.5 deg of back-off, and only the
  reset button re-arms it, so the release lasts for the trial, as in sim.

The shaft's friction washer is set so the wheel starts turning at 0.75 N pulled at a lobe tip, 8 cm out (0.06 N*m).

Reset:
1. Turn the wheel back about 10 deg.
2. Stand the bottle upright, cap up, on its 4 x 4 cm outline marked on the deck (base centre x 33.7, y -6.6,
   yaw 60 deg). Put a 1.5 mm shim between its outer face and guide segment 3.
3. Turn the wheel forward to 0: the index mark on the scale's 0 line, within 0.5 deg.
4. Press reset to arm the magnet.
5. Remove the shim.

## Real-rig calibration (at the start of each session)

Tools: a 0-10 N spring scale (0.1 N divisions), a dial indicator (0.01 mm) on a stand on the table, and the degree
scale.

1. **Hold.** With the wheel at 0 and the magnet armed, hook the scale to the cap and pull straight up, smoothly over
   about 2 s. The bottle must lift off at 7.5 +/- 0.5 N (the 6 N hold plus its 1.5 N weight). Adjust the coil voltage
   until it does. A holding magnet resists sliding less than lifting, so also record the sideways hold: pull at the
   base, toward the open side of the pocket. Any 3 mm move before the back-off is scored 0, on the rig as in sim.
2. **Release angle.** Turn the wheel back slowly by pushing the -90 deg lobe, as the demo does. The relay must drop
   (an LED across the coil goes out) between 6.0 and 6.5 deg on the scale, never before the scorer's 6 deg. Turning
   the wheel 3 deg forward must not trip it. Move the cam flag until both hold.
3. **The release does not move the bottle.** Re-arm, and put the indicator's plunger on the bottle's face toward the
   0 deg lobe, at mid-height. Turn the wheel back from 0 to 10 deg, through the release. The reading must change by
   under 0.5 mm, and the bottle must stay upright (the scoring limits are 3 mm and 5 deg). Then lift the bottle with
   the scale: it must come up at under 1.7 N (its weight plus under 0.2 N of residual magnetism).

## How the 6 deg release was checked in sim

The release can't move the bottle in sim, by construction. This was checked offline from the code and from a sim
trace:

- **Nothing touches the bottle during the back-off.** The pinch is a bond from the bottle to the world (`kit.bond("pinch", ...)`),
  so the bottle cannot move under 6 N. The `_open_pocket` step hook switches the bond off at 6 deg back. In plan, the
  trailing lobe's clearance to the bottle grows from 1.7 mm at 0 deg to 7.6 mm at 6 deg. The other lobes stay at least
  11 mm clear from 0 to 20 deg, and the guide segments 1.5 mm or more. `_geometry_check()` in the self-test asserts the
  lobe and guide clearances over 0 to 20 deg.
- **The scorer and the release use the same reading.** `_history()` sets the backed-off latch on the same 6 deg
  reading that releases the bond. The 3 mm / 5 deg pull check stops at that step.
- **Sim trace.** In a development sim run of the scripted push, the wheel passed 6 deg at 6.05 deg. The bottle was
  then 0.01 mm and 0.00 deg from its jam pose, and stayed so until the gripper took it. The wheel came to rest
  8.8 deg back.
- **The demo recording.** The PR's demo scores 1.0, and a 1.0 is impossible if the bottle moved 3 mm or tipped 5 deg
  before the wheel was 6 deg back. So the recording itself shows the release without the bottle moving past the
  limit.
