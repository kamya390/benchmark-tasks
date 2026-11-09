# Snap two wireless earbuds home in their charging case, latch the lid and tray the case

Instruction given to the policy: `Pack the earbuds in their case and put the closed case in its tray.`

Files: `task.json`, `earbud_case_snap_pack.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/earbud-case-snap-pack/earbud_case_snap_pack.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 16 cases pass (lid meets a proud head at 34.5 deg; one 1.5 mm up holds it at 12.6 deg)
left: 46.4 s plan, worst IK 0.090 mm / 0.013 deg
right: 46.4 s plan, worst IK 0.097 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/earbud-case-snap-pack/earbud_case_snap_pack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pack the earbuds in their case and put the closed case in its tray." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/earbud-case-snap-pack-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/earbud-case-snap-pack/earbud_case_snap_pack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pack the earbuds in their case and put the closed case in its tray." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/earbud-case-snap-pack-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 46 s of plan (about 1128 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/earbud-case-snap-pack-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/earbud-case-snap-pack-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Release each bud's catch and lift it back to 4.5 mm proud, open the lid to its 115 deg stop, and stand the case back in the packing nest. A jammed bud is a reject on a real line: replace it.
