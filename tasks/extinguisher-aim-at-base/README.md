# Put out a waste-bin fire with a portable extinguisher

Instruction given to the policy: `The waste bin is on fire. Put the fire out with the extinguisher, then set the extinguisher down on the grey mat.`

Files: `task.json`, `extinguisher_aim_at_base.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/extinguisher-aim-at-base/extinguisher_aim_at_base.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 15 cases pass
left: 29.7 s plan, worst IK 0.092 mm / 0.012 deg
right: 29.7 s plan, worst IK 0.087 mm / 0.012 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/extinguisher-aim-at-base/extinguisher_aim_at_base.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The waste bin is on fire. Put the fire out with the extinguisher, then set the extinguisher down on the grey mat." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/extinguisher-aim-at-base-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/extinguisher-aim-at-base/extinguisher_aim_at_base.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The waste bin is on fire. Put the fire out with the extinguisher, then set the extinguisher down on the grey mat." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/extinguisher-aim-at-base-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 30 s of plan (about 794 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/extinguisher-aim-at-base-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/extinguisher-aim-at-base-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Push the pin back in, stand the extinguisher on the mat with its lever up, stand the three flame blocks back up on the paper and re-bond them; the bin and mat do not move. On a real rig the flames would be a water-mist or LED fire trainer whose sensor reads where the agent lands.

## What is simulated and what is emulated

- Simulated (rigid-body physics): the extinguisher body, its sprung squeeze lever (a hinge held 15 deg open by its
  spring), the safety pin (a slider with light friction whose rod physically blocks the lever at about 11 deg until it
  is pulled out), the bin, the paper fuel bed, the mat and both arms.
- Not simulated: the extinguishing agent and the fire. Nothing is sprayed and nothing burns.
- Emulated, for scoring: while the pin is out (3.5 cm or more) and the lever is squeezed to within 8 deg of closed,
  the unit counts as discharging, for at most 10 s in all. Every control step the nozzle's axis is followed as a
  straight ray to the paper's surface. The step counts on a third of the bin only if the ray clears the near rim and
  lands on the paper between the near wall and the flames' far side, which is the base of the fire. A ray aimed at the
  flames lands past them, against the far wall, and is wasted charge.
- Emulated, for the viewer: the three orange flame blocks are glued upright. Once a third has had 1.2 s of discharge
  on its base, the score rule lets that flame block go. It is off balance, so it topples and lies flat on the paper,
  below the rim. A third that is out is visible on the video as a flame that has fallen.
