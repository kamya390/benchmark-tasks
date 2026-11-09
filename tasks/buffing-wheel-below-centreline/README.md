# Polish both faces of a blank on a running buffing wheel without letting it snatch

Instruction given to the policy: `Polish both faces of the aluminium blank on the buffing wheel, then stand it back in its rack.`

Files: `task.json`, `buffing_wheel_below_centreline.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `mechkit.py` (helper module imported by the task file: the two wheels' spindle joints and the
per-episode hooks; it patches yam_arena in memory at import, nothing on disk changes; written for this task set, no
third-party code), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/buffing-wheel-below-centreline/buffing_wheel_below_centreline.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 17 cases pass
right: 44.0 s plan, worst IK 0.097 mm / 0.012 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/buffing-wheel-below-centreline/buffing_wheel_below_centreline.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Polish both faces of the aluminium blank on the buffing wheel, then stand it back in its rack." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/buffing-wheel-below-centreline-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/buffing-wheel-below-centreline/buffing_wheel_below_centreline.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Polish both faces of the aluminium blank on the buffing wheel, then stand it back in its rack." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/buffing-wheel-below-centreline-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 44 s of plan (about 1080 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/buffing-wheel-below-centreline-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/buffing-wheel-below-centreline-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the blank back in its rack, face B away from the buffer; the wheels keep running (on a real rig: restart the buffer); the buffer, rests and rack do not move.
