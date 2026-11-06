# Close an air circuit breaker by hand when its motor operator is out of service

Instruction given to the policy: `This de-energised training breaker's motor operator is out of service. Close the breaker by hand.`

Files: `task.json`, `acb_charge_before_close.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/acb-charge-before-close/acb_charge_before_close.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 13 cases pass
left: 33.8 s plan, worst IK 0.075 mm / 0.016 deg
right: 33.8 s plan, worst IK 0.084 mm / 0.012 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/acb-charge-before-close/acb_charge_before_close.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This de-energised training breaker's motor operator is out of service. Close the breaker by hand." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/acb-charge-before-close-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/acb-charge-before-close/acb_charge_before_close.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This de-energised training breaker's motor operator is out of service. Close the breaker by hand." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/acb-charge-before-close-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 34 s of plan (about 876 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/acb-charge-before-close-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/acb-charge-before-close-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (1 minute at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Open the breaker and discharge its closing spring (on a real mock-up: reset the ratchet counter and both indicator flags); the handle springs back level by itself.
