# Take a heatsink off a processor without pulling the processor out of its socket

Instruction given to the policy: `Take the heatsink off the processor and put it on the grey mat. The processor stays in its socket.`

Files: `task.json`, `heatsink_twist_free.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `mechkit.py` (helper module imported by the task file: the processor's and the heatsink's breakable bonds;
it patches yam_arena in memory at import, nothing on disk changes; written for this task set, no third-party code),
`README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/heatsink-twist-free/heatsink_twist_free.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 22 cases pass
left: 21.4 s plan, worst IK 0.098 mm / 0.015 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/heatsink-twist-free/heatsink_twist_free.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the heatsink off the processor and put it on the grey mat. The processor stays in its socket." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/heatsink-twist-free-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/heatsink-twist-free/heatsink_twist_free.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the heatsink off the processor and put it on the grey mat. The processor stays in its socket." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/heatsink-twist-free-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 21 s of plan (about 628 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/heatsink-twist-free-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/heatsink-twist-free-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Seat the processor in the socket, set the heatsink square on it and re-bond it (on a real rig: a fresh layer of thermal putty, or a magnet-held processor coupon and a putty-bonded heatsink); the board, socket and mat do not move.
