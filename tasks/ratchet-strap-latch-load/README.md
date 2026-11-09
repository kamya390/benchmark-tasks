# Ratchet a tie-down strap taut round a crate and latch its handle before the deck is jolted

Instruction given to the policy: `Strap the crate down to the deck.`

Files: `task.json`, `ratchet_strap_latch_load.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/ratchet-strap-latch-load/ratchet_strap_latch_load.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 18 cases pass (demo winds 4 teeth: 60 N in the strap)
left: 34.1 s plan, worst IK 0.090 mm / 0.013 deg
right: 34.1 s plan, worst IK 0.078 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/ratchet-strap-latch-load/ratchet_strap_latch_load.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Strap the crate down to the deck." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/ratchet-strap-latch-load-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/ratchet-strap-latch-load/ratchet_strap_latch_load.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Strap the crate down to the deck." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/ratchet-strap-latch-load-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 34 s of plan (about 882 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/ratchet-strap-latch-load-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/ratchet-strap-latch-load-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Pull the ratchet's release and pay the strap out to 12 mm off the crate, set the handle 40 deg open, stand the crate against the stake board and run the deck back onto its rear stop.
