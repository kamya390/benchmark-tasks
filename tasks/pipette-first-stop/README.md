# Transfer one pipette volume with an air-displacement pipette

Instruction given to the policy: `Transfer one pipette volume of the blue sample from the reservoir into the empty tube, then stand the pipette back in its stand.`

Files: `task.json`, `pipette_first_stop.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/pipette-first-stop/pipette_first_stop.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 18 cases pass
left: 46.4 s plan, worst IK 0.087 mm / 0.012 deg
right: 46.4 s plan, worst IK 0.092 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/pipette-first-stop/pipette_first_stop.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Transfer one pipette volume of the blue sample from the reservoir into the empty tube, then stand the pipette back in its stand." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/pipette-first-stop-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/pipette-first-stop/pipette_first_stop.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Transfer one pipette volume of the blue sample from the reservoir into the empty tube, then stand the pipette back in its stand." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/pipette-first-stop-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 46 s of plan (about 1128 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/pipette-first-stop-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/pipette-first-stop-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the pipette in its stand with the button up; glue the reservoir's level marker and the tube's EMPTY tag back in place. Reservoir, stand and rack do not move. On a real rig the reservoir and tube would hold dyed water and the delivered volume be weighed, as a cross-check.

## What is simulated and what is emulated

- Simulated (rigid-body physics): the pipette body, the plunger on a sprung slider (12 mm soft stroke to the first
  stop), the blowout collar on a stiff sprung slider (8 mm more to the second stop), the stand, reservoir, tube and
  rack, and both arms.
- Not simulated: the liquid. There is no fluid, no volume and no air in the tip.
- Emulated, for scoring: aspiration and dispensing are read every control step from geometry a trainer would also
  check on video: the plunger's travel (button height against the body), the tip's position against the
  reservoir's sample surface (the blue band, 2 cm up) and against the tube, and the pipette's tilt.
- Emulated, for the viewer: when a volume is drawn up, the blue level marker on the reservoir's sight gauge is let
  go and drops onto its ledge; when a volume is dispensed, the tube's white EMPTY tag is let go and drops away,
  showing the blue window behind it. Both markers are plain parts glued in place that the score rule releases; they
  carry no physics of the liquid.
