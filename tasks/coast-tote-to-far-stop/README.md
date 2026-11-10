# Send a tote down a roller conveyor to the far station without slamming the end stop

Instruction given to the policy: `Send the tote down the conveyor to the packing station at the far end.`

Files: `task.json`, `coast_tote_to_far_stop.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/coast-tote-to-far-stop/coast_tote_to_far_stop.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
0.00  tote rear stopped at x 0.175  | start
   1.00  tote rear stopped at x 0.885  | demo push to 0.5 m/s: rolls to rest at the station
   1.00  tote rear stopped at x 0.874  | slow end of the window (0.47 m/s)
   1.00  tote rear stopped at x 0.885  | fast end of the window (0.58 m/s): meets the stop gently, bottle stands
   0.00  tote rear stopped at x 0.457  | too soft (0.30 m/s): stops in the lane
   0.25  tote rear stopped at x 0.736  | a bit soft (0.42 m/s): 0.25
   0.50  tote rear stopped at x 0.758  | nearly (0.43 m/s): within 5 cm, 0.5
   0.00  tote rear stopped at x 0.885  | shoved (0.85 m/s): slams the stop, bottle falls
   0.00  tote rear stopped at x 0.885  | shoved hard from a standing start (0.70 m/s)
   0.00  tote rear stopped at x 0.340  | pushed with the hand pointing down as far as it reaches (rear to 0.34), let go
   0.50  tote rear stopped at x 0.760  | walked by hand, pointing forward, to its reach limit (rear 0.76), let go
   0.00  tote rear stopped at x 0.760  | walked by hand to the reach limit and held there
   0.00  tote rear stopped at x 0.850  | tote lifted and carried to the station
   0.00  tote rear stopped at x 0.850  | bottle lifted out and set back in the tote, then the tote sent to the station
   0.00  tote rear stopped at x 0.850  | bottle knocked over at the start, set upright again, then the tote sent to the station
   0.00  tote rear stopped at x 0.885  | demo, but a gripper is left near the tote
   0.00  tote rear stopped at x 0.850  | tote still rolling at the last step
scoring: 18 cases pass; release-speed window 0.450-0.629 m/s, demo 0.5 m/s
left: 15.2 s plan, worst IK 0.090 mm / 0.013 deg
right: 15.2 s plan, worst IK 0.092 mm / 0.015 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/coast-tote-to-far-stop/coast_tote_to_far_stop.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Send the tote down the conveyor to the packing station at the far end." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/coast-tote-to-far-stop-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/coast-tote-to-far-stop/coast_tote_to_far_stop.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Send the tote down the conveyor to the packing station at the far end." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/coast-tote-to-far-stop-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 15 s of plan (about 504 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/coast-tote-to-far-stop-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/coast-tote-to-far-stop-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Roll the tote back to the near end of the conveyor (rear face 17.5 cm ahead of the arm bases) and stand the bottle upright in its middle.
