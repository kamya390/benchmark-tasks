# Drill a deep hole in pecks, clearing the chips

Instruction given to the policy: `Drill the hole in the steel block down to the depth stop, then bring the drill back up.`

Files: `task.json`, `peck_drill_deep_hole.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/peck-drill-deep-hole/peck_drill_deep_hole.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 15 cases pass
right: 22.5 s plan, worst IK 0.086 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/peck-drill-deep-hole/peck_drill_deep_hole.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Drill the hole in the steel block down to the depth stop, then bring the drill back up." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/peck-drill-deep-hole-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/peck-drill-deep-hole/peck_drill_deep_hole.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Drill the hole in the steel block down to the depth stop, then bring the drill back up." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/peck-drill-deep-hole-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 22 s of plan (about 650 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/peck-drill-deep-hole-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/peck-drill-deep-hole-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Let the quill return to its top stop and refit the drill in the chuck (on a real rig: a new drill and a fresh block, or a bore plug and a depth-scale check); the block, vise and press do not move.
