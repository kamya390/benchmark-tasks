# Refit a small engine's foam air-filter element so it seals all round

Instruction given to the policy: `Fit the clean foam filter element into the air-cleaner base so the cover can go on, then let go.`

Files: `task.json`, `foam_filter_seated_all_round.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`
and an offline self-test), `softkit.py` and `mechkit.py` (helper modules, see below), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/foam-filter-seated-all-round/foam_filter_seated_all_round.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 13 cases pass
left: 47.0 s plan, worst IK 0.090 mm / 0.013 deg
right: 47.0 s plan, worst IK 0.093 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/foam-filter-seated-all-round/foam_filter_seated_all_round.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the clean foam filter element into the air-cleaner base so the cover can go on, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/foam-filter-seated-all-round-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/foam-filter-seated-all-round/foam_filter_seated_all_round.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the clean foam filter element into the air-cleaner base so the cover can go on, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/foam-filter-seated-all-round-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 47 s of plan (about 1140 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/foam-filter-seated-all-round-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/foam-filter-seated-all-round-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Lift the element out, let it recover its shape, and lay it back on the base centred over the pocket.

## Helper modules shipped with the task

- `softkit.py` (v0.1): puts a real deformable body in a task file. A `softkit.Soft` entry in `OBJECTS` is spawned as
  an Isaac Lab-Arena `DeformableObject` (PhysX FEM volume for `kind="block"` / `"ball"`, PhysX surface deformable for
  `kind="cloth"`), and `score()` receives its mesh nodes (`softkit.nodes(state)`; `softkit.motion(state, name)` measures stillness from node displacement). It hooks
  into yam-arena in memory at import time (no files are changed or installed) and works offline on hand-built node
  sets for the self-test (`softkit.rest_nodes`, `softkit.fake`).
- `mechkit.py` (v0.2): provides the per-episode memory (cleared at every reset) and the grippers' TCP positions
  that `softkit.py` and the scorer use. No joints or step rules are used in this task.
- Both are imported by the task file from its own folder; the commands above run them as they are.

To see the element's mesh size and the hook working, the stdout of a run prints one line per soft body:

```
[softkit] element: 265 nodes
```

## Assets and licences

None. The housing is built from yam-arena boxes and the foam element's mesh is generated in code at spawn time
(Isaac Lab's box mesh, tetrahedralized by the simulator). No third-party model, texture or mesh file is used.
