# Get a loaded cart over a raised sill by tipping it back onto its big wheels, not ramming its casters

Instruction given to the policy: `Roll the loaded cart into the bay, up against its back stop, with the carton still on it, then let go. Do not lift the cart clear of the floor.`

Files: `task.json`, `cart_casters_over_sill.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `mechkit.py` (the joint kit the task imports: it adds the wheels' free axles and the handle's swivel to the stock
yam_arena runtime in memory, nothing to install), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/cart-casters-over-sill/cart_casters_over_sill.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 18 cases pass
right: 26.4 s plan, worst IK 0.096 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/cart-casters-over-sill/cart_casters_over_sill.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Roll the loaded cart into the bay, up against its back stop, with the carton still on it, then let go. Do not lift the cart clear of the floor." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cart-casters-over-sill-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/cart-casters-over-sill/cart_casters_over_sill.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Roll the loaded cart into the bay, up against its back stop, with the carton still on it, then let go. Do not lift the cart clear of the floor." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cart-casters-over-sill-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 26 s of plan (about 728 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/cart-casters-over-sill-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/cart-casters-over-sill-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Roll the cart back off the platform (tip it back over the sill) and set it square on its mark with the casters 3 cm short of the sill, the handle level, and the carton centred on the deck; the platform and stop do not move.
