# Clean a coil's strainer and put its balancing valve back in the notch it was in

Instruction given to the policy: `Swap the strainer's clogged screen for the clean one in the rack, then put the coil back in service.`

Files: `task.json`, `balancing_valve_back_to_its_notch.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/balancing-valve-back-to-its-notch/balancing_valve_back_to_its_notch.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 19 cases pass
left: 61.7 s plan, worst IK 0.082 mm / 0.013 deg
right: 61.7 s plan, worst IK 0.080 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/balancing-valve-back-to-its-notch/balancing_valve_back_to_its_notch.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Swap the strainer's clogged screen for the clean one in the rack, then put the coil back in service." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/balancing-valve-back-to-its-notch-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/balancing-valve-back-to-its-notch/balancing_valve_back_to_its_notch.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Swap the strainer's clogged screen for the clean one in the rack, then put the coil back in service." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/balancing-valve-back-to-its-notch-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 62 s of plan (about 1434 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/balancing-valve-back-to-its-notch-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/balancing-valve-back-to-its-notch-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Shut the valve, put the clogged cartridge back in the strainer and the clean one in the rack, empty the tray, then set the lever back in the third notch (30 deg).
