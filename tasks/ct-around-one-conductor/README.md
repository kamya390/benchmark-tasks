# Fit a split-core current sensor so it measures the circuit's current

Instruction given to the policy: `This circuit runs from the grey breaker panel on the left to the PDU on the right. Fit the split-core current sensor so that it measures the current the PDU draws, then let go.`

Files: `task.json`, `ct_around_one_conductor.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/ct-around-one-conductor/ct_around_one_conductor.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 22 cases pass
left: 35.2 s plan, worst IK 0.043 mm / 0.017 deg
right: 35.2 s plan, worst IK 0.091 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/ct-around-one-conductor/ct_around_one_conductor.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This circuit runs from the grey breaker panel on the left to the PDU on the right. Fit the split-core current sensor so that it measures the current the PDU draws, then let go." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/ct-around-one-conductor-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/ct-around-one-conductor/ct_around_one_conductor.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This circuit runs from the grey breaker panel on the left to the PDU on the right. Fit the split-core current sensor so that it measures the current the PDU draws, then let go." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/ct-around-one-conductor-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 35 s of plan. In the recorded run the catch
latched and the success hold ended the episode at step 656, inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/ct-around-one-conductor-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/ct-around-one-conductor-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (1.5 minutes at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Let the jaw hang open at its rest angle (39 degrees) with the catch hook home on its stop, and hang the sensor by its notch on the parking peg, arrow pointing away from the arms; the conductors, panel and PDU do not move.
