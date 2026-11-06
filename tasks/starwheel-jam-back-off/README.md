# Free a bottle jammed in a capper's infeed starwheel

Instruction given to the policy: `The capper's infeed has jammed on a bottle and the line is stopped and locked out. Take the jammed bottle out of the starwheel and put it in the red reject bin.`

Files: `task.json`, `starwheel_jam_back_off.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/starwheel-jam-back-off/starwheel_jam_back_off.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 18 cases pass
right: 29.7 s plan, worst IK 0.094 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/starwheel-jam-back-off/starwheel_jam_back_off.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The capper's infeed has jammed on a bottle and the line is stopped and locked out. Take the jammed bottle out of the starwheel and put it in the red reject bin." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/starwheel-jam-back-off-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/starwheel-jam-back-off/starwheel_jam_back_off.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The capper's infeed has jammed on a bottle and the line is stopped and locked out. Take the jammed bottle out of the starwheel and put it in the red reject bin." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/starwheel-jam-back-off-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 30 s of plan (about 794 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/starwheel-jam-back-off-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/starwheel-jam-back-off-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Turn the wheel back to its index (0 deg), stand the bottle back in the pocket against the guide and re-wedge it (on a real rig: a 2 mm interference fit, or a 6 N magnet under the pocket); the deck, guide, chevrons, shaft, isolator and bin do not move.
