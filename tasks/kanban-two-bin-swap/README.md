# Swap an empty kanban tote for the full one

Instruction given to the policy: `The grey tote in the pick slot is empty. Put its yellow kanban card in the white card post, put the empty tote on the black return mat, then move the full blue tote forward into the pick slot between the rails.`

Files: `task.json`, `kanban_two_bin_swap.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/kanban-two-bin-swap/kanban_two_bin_swap.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 14 cases pass
left: 49.5 s plan, worst IK 0.091 mm / 0.018 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/kanban-two-bin-swap/kanban_two_bin_swap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The grey tote in the pick slot is empty. Put its yellow kanban card in the white card post, put the empty tote on the black return mat, then move the full blue tote forward into the pick slot between the rails." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/kanban-two-bin-swap-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/kanban-two-bin-swap/kanban_two_bin_swap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The grey tote in the pick slot is empty. Put its yellow kanban card in the white card post, put the empty tote on the black return mat, then move the full blue tote forward into the pick slot between the rails." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/kanban-two-bin-swap-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 50 s of plan (about 1190 steps
including the success hold), inside the run limit of 2,400 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/kanban-two-bin-swap-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/kanban-two-bin-swap-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 2,400 steps (2 minutes at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Put the grey tote back in the pick slot with the card on its lid and the blue tote in the reserve slot.
