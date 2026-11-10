# Jog a desktop CNC router's cutter over a cross mark with its spring-centred joystick

Instruction given to the policy: `Jog the router's cutter over the cross mark.`

Files: `task.json`, `cnc_jog_joystick_to_mark.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/cnc-jog-joystick-to-mark/cnc_jog_joystick_to_mark.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
0.00  cutter 120.00 mm from the mark, closest to the clamp 127.00 mm  | start
   1.00  cutter   0.11 mm from the mark, closest to the clamp   6.89 mm  | demo (the planned tilts with the arm's give)
   0.00  cutter   7.00 mm from the mark, closest to the clamp  -0.00 mm  | naive: full speed until the cutter is over the mark, then let go: runs on into the clamp
   0.00  cutter   7.00 mm from the mark, closest to the clamp  -0.00 mm  | naive with a 0.3 s reaction time
   0.25  cutter   6.00 mm from the mark, closest to the clamp   1.00 mm  | naive, let go 4 mm before the mark: stops 6 mm past it
   0.00  cutter   7.00 mm from the mark, closest to the clamp  -0.00 mm  | naive, let go 3 mm before the mark: runs into the clamp
   1.00  cutter   1.00 mm from the mark, closest to the clamp   6.00 mm  | careful: full speed to 25 mm short, let go, then short gentle nudges
   1.00  cutter   0.25 mm from the mark, closest to the clamp   7.25 mm  | careful, but slower nudges at 3.5 deg
   0.75  cutter   0.11 mm from the mark, closest to the clamp   6.89 mm  | demo tilts, but the hand stays on the knob at the end
   0.50  cutter   2.75 mm from the mark, closest to the clamp   9.75 mm  | stopped about 3 mm short of the mark
   0.00  cutter  14.00 mm from the mark, closest to the clamp  21.00 mm  | stopped about 14 mm short (eased off early, never finished)
   0.25  cutter   6.50 mm from the mark, closest to the clamp  13.50 mm  | stopped about 7 mm short
   0.00  cutter  15.23 mm from the mark, closest to the clamp  -0.00 mm  | overshoot into the clamp, then jogged back onto the mark
   0.00  cutter   0.11 mm from the mark, closest to the clamp  13.89 mm  | demo, but the clamp was knocked loose by a gripper (7 mm) on the way
   0.00  cutter   0.00 mm from the mark, closest to the clamp   7.00 mm  | the gantry held on the mark by hand (gripper on it), stick never used
   0.00  cutter   1.48 mm from the mark, closest to the clamp   8.48 mm  | the cutter still moving at 8 mm/s near the mark at the last step
   1.00  cutter   1.08 mm from the mark, closest to the clamp   5.92 mm  | jogged back first (the wrong way), then the careful way
   0.75  cutter   0.11 mm from the mark, closest to the clamp   6.89 mm  | stick held tilted 1.5 deg (inside the dead band) by the hand on the mark
scoring: 19 cases pass
left: 16.5 s plan, worst IK 0.090 mm / 0.013 deg
right: 16.5 s plan, worst IK 0.079 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/cnc-jog-joystick-to-mark/cnc_jog_joystick_to_mark.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Jog the router's cutter over the cross mark." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cnc-jog-joystick-to-mark-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/cnc-jog-joystick-to-mark/cnc_jog_joystick_to_mark.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Jog the router's cutter over the cross mark." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cnc-jog-joystick-to-mark-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 16 s of plan (about 530 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/cnc-jog-joystick-to-mark-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/cnc-jog-joystick-to-mark-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Jog the gantry back so the cutter is 12 cm behind the mark (x 30), check the stick is centred, and check the clamp is tight on the work. A clamp that was struck is re-tightened and the cutter is replaced.
