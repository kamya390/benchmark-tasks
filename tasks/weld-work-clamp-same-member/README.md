# Clamp the welder's work lead to the cracked arm, not across its pivot

Instruction given to the policy: `The crack marked orange on the lift arm is going to be welded. Put the welder's work clamp on for the repair, then let go.`

Files: `task.json`, `weld_work_clamp_same_member.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/weld-work-clamp-same-member/weld_work_clamp_same_member.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 16 cases pass (jaws 11.2 deg on a fin; squeeze to 39.1 mm across the handles)
left: 18.2 s plan, worst IK 0.090 mm / 0.013 deg
right: 18.2 s plan, worst IK 0.078 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/weld-work-clamp-same-member/weld_work_clamp_same_member.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The crack marked orange on the lift arm is going to be welded. Put the welder's work clamp on for the repair, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/weld-work-clamp-same-member-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/weld-work-clamp-same-member/weld_work_clamp_same_member.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The crack marked orange on the lift arm is going to be welded. Put the welder's work clamp on for the repair, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/weld-work-clamp-same-member-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 18 s of plan (about 564 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/weld-work-clamp-same-member-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/weld-work-clamp-same-member-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Hang the work clamp back on its bracket fin with its jaws closed on it; the frame and lift arm do not move.
