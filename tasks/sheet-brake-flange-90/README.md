# Bend a sheet's flange to 90 degrees in a hand brake

Instruction given to the policy: `Bend the sheet's flange up to 90 degrees on the hand brake, then put the finished bracket on the grey tray.`

Files: `task.json`, `sheet_brake_flange_90.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/sheet-brake-flange-90/sheet_brake_flange_90.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 18 cases pass
left: 59.8 s plan, worst IK 0.095 mm / 0.013 deg
right: 59.8 s plan, worst IK 0.090 mm / 0.018 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/sheet-brake-flange-90/sheet_brake_flange_90.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Bend the sheet's flange up to 90 degrees on the hand brake, then put the finished bracket on the grey tray." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/sheet-brake-flange-90-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/sheet-brake-flange-90/sheet_brake_flange_90.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Bend the sheet's flange up to 90 degrees on the hand brake, then put the finished bracket on the grey tray." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/sheet-brake-flange-90-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 60 s of plan (about 1396 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/sheet-brake-flange-90-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/sheet-brake-flange-90-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Flatten the blank and lay it back on the bed against the gauge (on a real rig: a fresh blank), lower the leaf flush and raise the clamping bar to its upper stop; the bed, gauge, posts and tray do not move.
