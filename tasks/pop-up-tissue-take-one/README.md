# Take one tissue from a pop-up box and leave the next one standing ready

Instruction given to the policy: `Take one tissue out of the box and put it on the tray, leaving the box in place with the next tissue ready to pull.`

Files: `task.json`, `pop_up_tissue_take_one.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`
and an offline self-test), `softkit.py` and `mechkit.py` (helper modules, see below), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/pop-up-tissue-take-one/pop_up_tissue_take_one.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). The scoring cases include a brief 6 mm lift and a brief
6 deg tilt of the box that then settles on its spot (still full credit), and a tip-over, a box carried more than 5 cm
off the table and a box knocked off the table (0, latched for the trial). Expected output:

```
scoring: 21 cases pass
left: 27.0 s plan, worst IK 0.075 mm / 0.012 deg
right: 27.0 s plan, worst IK 0.095 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/pop-up-tissue-take-one/pop_up_tissue_take_one.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take one tissue out of the box and put it on the tray, leaving the box in place with the next tissue ready to pull." \
  --scorer task_score --max-steps 700 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/pop-up-tissue-take-one-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

Scoring in short (full text in `task.json`): a trial is zeroed for good only by a consequential event (the box tilted
more than 45 deg, its lowest corner more than 5 cm above the table or its bottom more than 2 cm below the table top,
or the second tissue out of the box too). A brief lift or tilt short of that costs nothing; at the end the box must be
back within 1.5 cm of its spot, turned less than 10 deg, tilted less than 5 deg and resting on the table.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/pop-up-tissue-take-one/pop_up_tissue_take_one.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take one tissue out of the box and put it on the tray, leaving the box in place with the next tissue ready to pull." \
  --scorer task_score --max-steps 700 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/pop-up-tissue-take-one-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 27 s of plan; the episode ends on success about
1 s after the tissue is let go on the tray (about 450 steps), inside the run limit of 700 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/pop-up-tissue-take-one-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/pop-up-tissue-take-one-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 700 steps (35 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Take the second tissue back down, re-interfold the two sheets as at the start (the top one round the next one's edge, its end standing 2.5 cm up through the slit) and put the box back on its spot; empty the tray.

## Helper modules shipped with the task

- `softkit.py` (v0.2): puts real deformable bodies in a task file. A `softkit.Soft` entry in `OBJECTS` is spawned as
  an Isaac Lab-Arena `DeformableObject` (here `kind="cloth"`: a PhysX surface deformable); v0.2 adds `profile`, a
  creased sheet given by its cross-section, spawned about its node centroid. `score()` receives the mesh nodes
  (`softkit.nodes(state)`; `softkit.motion(state, name)` measures stillness from node displacement). It hooks into
  yam-arena in memory at import time (no files are changed or installed) and works offline on hand-built node sets
  for the self-test (`softkit.rest_nodes`, `softkit.fake`).
- `mechkit.py` (v0.2): the tissue box (one free rigid body made of boxes) and its sprung film strip (a prismatic
  joint with a spring, authored as a plain PhysX joint), plus the per-episode memory and the grippers' TCP positions.
  No step rules are used in this task.
- Both are imported by the task file from its own folder; the commands above run them as they are.

The stdout of a run prints one line per soft body:

```
[softkit] tissue_1: 675 nodes
[softkit] tissue_2: 725 nodes
```

## Assets and licences

None. The box, film strip and tray are built from yam-arena boxes and the tissues' meshes are generated in code at
spawn time. No third-party model, texture or mesh file is used.
