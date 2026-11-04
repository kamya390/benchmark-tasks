# Unload a finished part and load a raw blank into the machine nest

Instruction given to the policy: `The part between the jaws of the machine's nest is finished. Unload it into the tray of finished parts, then load one raw blank into the nest so it lies on the nest floor between the two jaws, its length along them. Finished parts are machined 15 mm shorter than raw blanks. Never put a raw blank with the finished parts or a finished part with the raw blanks.`

Files: `task.json`, `machine_tending_swap.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/machine-tending-swap/machine_tending_swap.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 13 cases pass
left: 33.0 s plan, worst IK 0.092 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/machine-tending-swap/machine_tending_swap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The part between the jaws of the machine's nest is finished. Unload it into the tray of finished parts, then load one raw blank into the nest so it lies on the nest floor between the two jaws, its length along them. Finished parts are machined 15 mm shorter than raw blanks. Never put a raw blank with the finished parts or a finished part with the raw blanks." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/machine-tending-swap-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/machine-tending-swap/machine_tending_swap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The part between the jaws of the machine's nest is finished. Unload it into the tray of finished parts, then load one raw blank into the nest so it lies on the nest floor between the two jaws, its length along them. Finished parts are machined 15 mm shorter than raw blanks. Never put a raw blank with the finished parts or a finished part with the raw blanks." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/machine-tending-swap-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 33 s of plan (about 860 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/machine-tending-swap-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/machine-tending-swap-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Lay the finished part back between the jaws and the two blanks and the other finished part back in their trays as listed.
