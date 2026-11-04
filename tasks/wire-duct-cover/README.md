# Close the open run of a wire duct with a cut-to-length cover, two arms

Instruction given to the policy: `Pick up the loose grey duct cover with both arms, keep it level, and lay it on the wire duct in the open stretch between the two dark cover pieces already fitted at its ends, so it lies on both duct walls along its whole length and overhangs both walls equally. Then let go.`

Files: `task.json`, `wire_duct_cover.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/wire-duct-cover/wire_duct_cover.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 13 cases pass
left: 18.2 s plan, worst IK 0.085 mm / 0.013 deg
right: 18.2 s plan, worst IK 0.085 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/wire-duct-cover/wire_duct_cover.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick up the loose grey duct cover with both arms, keep it level, and lay it on the wire duct in the open stretch between the two dark cover pieces already fitted at its ends, so it lies on both duct walls along its whole length and overhangs both walls equally. Then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/wire-duct-cover-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/wire-duct-cover/wire_duct_cover.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick up the loose grey duct cover with both arms, keep it level, and lay it on the wire duct in the open stretch between the two dark cover pieces already fitted at its ends, so it lies on both duct walls along its whole length and overhangs both walls equally. Then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/wire-duct-cover-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 18 s of plan (about 564 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/wire-duct-cover-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/wire-duct-cover-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (1 minute at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Lay the loose cover back flat on its mark.
