# Move the perforated floor tile to the loaded rack's inlet, swapping it with a solid tile

Instruction given to the policy: `Perforated floor tiles (the light ones) let cold air up in front of a rack, so they belong in front of racks that are loaded with servers. Swap the tile in front of the empty rack with the tile in front of the loaded rack that has a solid tile (the dark one), using the white tile cart as a place to set a tile down. Lay each tile flat on its pedestal and leave the tile in front of the middle rack alone.`

Files: `task.json`, `perforated_tile_to_load.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/perforated-tile-to-load/perforated_tile_to_load.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 15 cases pass
left: 55.5 s plan, worst IK 0.093 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/perforated-tile-to-load/perforated_tile_to_load.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Perforated floor tiles (the light ones) let cold air up in front of a rack, so they belong in front of racks that are loaded with servers. Swap the tile in front of the empty rack with the tile in front of the loaded rack that has a solid tile (the dark one), using the white tile cart as a place to set a tile down. Lay each tile flat on its pedestal and leave the tile in front of the middle rack alone." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/perforated-tile-to-load-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/perforated-tile-to-load/perforated_tile_to_load.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Perforated floor tiles (the light ones) let cold air up in front of a rack, so they belong in front of racks that are loaded with servers. Swap the tile in front of the empty rack with the tile in front of the loaded rack that has a solid tile (the dark one), using the white tile cart as a place to set a tile down. Lay each tile flat on its pedestal and leave the tile in front of the middle rack alone." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/perforated-tile-to-load-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 56 s of plan (about 1310 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/perforated-tile-to-load-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/perforated-tile-to-load-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Lay the solid tile back on spot A and the perforated tiles on spots M and B; the pedestals, racks, servers and cart are fixed.
