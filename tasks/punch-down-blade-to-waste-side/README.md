# Punch a wire down on a 110 block with the tool's cutting side toward the loose end

Instruction given to the policy: `Punch the blue wire down into its contact on the 110 block with the punch-down tool, trimming off its loose end, then lift the tool off.`

Files: `task.json`, `punch_down_blade_to_waste_side.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/punch-down-blade-to-waste-side/punch_down_blade_to_waste_side.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 17 cases pass (conductor drops 1.83 mm from the entry vee to seated; fork tips 125.7 mm below the TCP)
left: 27.3 s plan, worst IK 0.074 mm / 0.013 deg
right: 27.3 s plan, worst IK 0.099 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/punch-down-blade-to-waste-side/punch_down_blade_to_waste_side.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Punch the blue wire down into its contact on the 110 block with the punch-down tool, trimming off its loose end, then lift the tool off." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/punch-down-blade-to-waste-side-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/punch-down-blade-to-waste-side/punch_down_blade_to_waste_side.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Punch the blue wire down into its contact on the 110 block with the punch-down tool, trimming off its loose end, then lift the tool off." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/punch-down-blade-to-waste-side-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 27 s of plan (about 746 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/punch-down-blade-to-waste-side-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/punch-down-blade-to-waste-side-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Lay a new conductor across position 5 (a punched conductor cannot be re-used: its insulation is slit), with its run in the fanning strip and its loose end 20 mm out over the front, and stand the tool back in its holster with the CUT mark toward the back.
