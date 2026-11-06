# Mount a spring-loaded rack rail between two posts at the marked rack position

Instruction given to the policy: `Mount the slide rail between the two rack posts at the rack position marked by the orange tab, then let go.`

Files: `task.json`, `rack_rail_sprung_span.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `mechkit.py` (the joint kit the task imports: it adds the rail's sprung prismatic slider to the stock
yam_arena runtime in memory, nothing to install), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/rack-rail-sprung-span/rack_rail_sprung_span.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 20 cases pass
left: 22.1 s plan, worst IK 0.081 mm / 0.013 deg
right: 22.1 s plan, worst IK 0.081 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/rack-rail-sprung-span/rack_rail_sprung_span.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Mount the slide rail between the two rack posts at the rack position marked by the orange tab, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/rack-rail-sprung-span-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/rack-rail-sprung-span/rack_rail_sprung_span.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Mount the slide rail between the two rack posts at the rack position marked by the orange tab, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/rack-rail-sprung-span-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 22 s of plan (about 642 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/rack-rail-sprung-span-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/rack-rail-sprung-span-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (1 minute at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Lift the rail out (pull both ends in), let the spring push it back to 30 cm, and lay it flat on its mark in front of the rack; the posts and tab do not move.
