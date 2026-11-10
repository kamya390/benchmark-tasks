# Give a manikin two breaths with a bag-valve mask, lifting the jaw into the mask

Instruction given to the policy: `Give the training manikin two breaths with the bag-valve mask.`

Files: `task.json`, `bvm_mask_seal_jaw_lift.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/bvm-mask-seal-jaw-lift/bvm_mask_seal_jaw_lift.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 19 cases pass (pinch stack 70.4 mm at jaw 0; gripper opens 94 mm)
left: 20.5 s plan, worst IK 0.096 mm / 0.013 deg
right: 20.5 s plan, worst IK 0.068 mm / 0.015 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/bvm-mask-seal-jaw-lift/bvm_mask_seal_jaw_lift.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Give the training manikin two breaths with the bag-valve mask." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/bvm-mask-seal-jaw-lift-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/bvm-mask-seal-jaw-lift/bvm_mask_seal_jaw_lift.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Give the training manikin two breaths with the bag-valve mask." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/bvm-mask-seal-jaw-lift-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 20 s of plan (about 610 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/bvm-mask-seal-jaw-lift-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/bvm-mask-seal-jaw-lift-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Lift the mask off and rest it back on the face, centred, let the jaw fall onto its slack stop, let the bag refill and the chest plate settle.
