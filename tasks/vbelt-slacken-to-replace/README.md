# Change a drive belt by sliding the motor in to slacken it, then tension the new belt

Instruction given to the policy: `Replace the worn red belt on this drive with the new black belt from the rests, put the worn belt in the red bin, and leave the drive ready to run.`

Files: `task.json`, `vbelt_slacken_to_replace.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `mechkit.py` (helper module imported by the task file: the motor base's friction slideway and the multi-box pulleys and belts; it adds them to the stock
yam_arena runtime in memory, nothing to install), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/vbelt-slacken-to-replace/vbelt_slacken_to_replace.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 18 cases pass
right: 56.8 s plan, worst IK 0.090 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/vbelt-slacken-to-replace/vbelt_slacken_to_replace.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Replace the worn red belt on this drive with the new black belt from the rests, put the worn belt in the red bin, and leave the drive ready to run." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/vbelt-slacken-to-replace-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/vbelt-slacken-to-replace/vbelt_slacken_to_replace.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Replace the worn red belt on this drive with the new black belt from the rests, put the worn belt in the red bin, and leave the drive ready to run." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/vbelt-slacken-to-replace-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 57 s of plan (about 1336 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/vbelt-slacken-to-replace-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/vbelt-slacken-to-replace-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Slide the motor base back to its start mark (d = 0), lay the red belt on the deck round both hubs and the black belt on its rests; the channel, driven pulley, rests and bin do not move.
