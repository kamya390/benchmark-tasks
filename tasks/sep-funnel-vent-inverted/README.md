# Mix the two layers in a separatory funnel by shaking it

Instruction given to the policy: `Mix the two layers in the separatory funnel by shaking it, then put it back in its support with the stopper in.`

Files: `task.json`, `sep_funnel_vent_inverted.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `mechkit.py` (helper module imported by the task file: the stopcock key's friction hinge and the stopper's hold;
it patches yam_arena in memory at import, nothing on disk changes; written for this task set, no third-party code),
`README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/sep-funnel-vent-inverted/sep_funnel_vent_inverted.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 17 cases pass
left: 57.7 s plan, worst IK 0.092 mm / 0.014 deg, largest joint step 3.3 deg
right: 57.7 s plan, worst IK 0.079 mm / 0.015 deg, largest joint step 5.6 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/sep-funnel-vent-inverted/sep_funnel_vent_inverted.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Mix the two layers in the separatory funnel by shaking it, then put it back in its support with the stopper in." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/sep-funnel-vent-inverted-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/sep-funnel-vent-inverted/sep_funnel_vent_inverted.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Mix the two layers in the separatory funnel by shaking it, then put it back in its support with the stopper in." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/sep-funnel-vent-inverted-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 58 s of plan (about 1354 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/sep-funnel-vent-inverted-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/sep-funnel-vent-inverted-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the funnel back in its support with the stopcock shut and the stopper seated (re-bonded); on a real rig, refill the two layers (water under a coloured light oil) and reset the pressure counter, or use a funnel with a small sealed gas cartridge; the support does not move.
