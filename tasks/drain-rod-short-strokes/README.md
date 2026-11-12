# Rod out a blocked drain pipe without kinking the drain rod

Instruction given to the policy: `The grey pipe is blocked. Push the clog out of the far end of the pipe with the drain rod that is started in it, then let go of the rod.`

Files: `task.json`, `drain_rod_short_strokes.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/drain-rod-short-strokes/drain_rod_short_strokes.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 18 cases pass
left: 87.1 s plan, worst IK 0.076 mm / 0.016 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/drain-rod-short-strokes/drain_rod_short_strokes.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The grey pipe is blocked. Push the clog out of the far end of the pipe with the drain rod that is started in it, then let go of the rod." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/drain-rod-short-strokes-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/drain-rod-short-strokes/drain_rod_short_strokes.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The grey pipe is blocked. Push the clog out of the far end of the pipe with the drain rod that is started in it, then let go of the rod." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/drain-rod-short-strokes-demo
```

The demo is `demo()` in the task file. It is planned from the pipe's fixed geometry and reads one piece of
simulator state, the clog's travel, at the start of each 22 mm stroke: once the whole clog is 8 mm past the outlet it
stops feeding and lifts clear. How far each stroke drives the clog varies (the pads slip on some strokes; the physics
is not deterministic), so it allows up to 19 strokes, 87.1 s of plan; the full 19-stroke plan finishes by about 1,740 steps,
inside the run limit of 1,800 steps, and pad slip varies the stroke count from run to run.

## Videos and screenshot

```bash
for log in ~/yam-eval/drain-rod-short-strokes-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/drain-rod-short-strokes-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the physics is not bit-for-bit deterministic, so the demo checks the clog's travel before each stroke.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Pull the rod back out, push the clog back up the bore with the probe rod until its trailing face is 3.6 cm inside the mouth (probe depth 16.4 cm from the outlet to its leading face), and lay the rod back with its tip just inside the mouth and its tail in its curve; the board and pipe do not move.
