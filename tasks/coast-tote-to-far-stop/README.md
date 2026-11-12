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

## The conveyor (from the task file)

Positions in cm from the midpoint between the arm bases (x forward, y left).

| Part | Where and what | In `coast_tote_to_far_stop.py` |
|---|---|---|
| Kind | A non-powered gravity roller conveyor of the light-duty miniature kind used for small totes at bench stations, level, along y -20. | `LY` |
| Rollers | 23 zinc-plated steel tube rollers, 16 mm in diameter, at 40 mm centres from x 12.5 to x 100.5, 13 cm long between the side frames, on sealed ball bearings on fixed steel axles. The 14 cm tote always rides on 3 or 4 of them. In sim they are drawn only (no collision). | `ROLL_PITCH`, `ROLL_W` |
| Roller bed | Roller tops 3 cm above the table. In sim one flat bed with friction 0.02, combined by minimum, so the loaded tote slows at 0.02 x 9.81 = 0.196 m/s^2 (also measured in sim: 0.196 m/s^2). | `BED_H`, `MU_ROLL` |
| Frame | Steel, x 10.5 to 104, running past the table's edge (x 91) onto a steel leg. Side rails 1.2 cm wide, 13 cm apart, 2 cm above the roller tops. | `X_IN`, `X_END`, `LANE_W`, `RAIL_W`, `RAIL_UP` |
| End stop | Red steel, face at x 102.5, 4.5 cm above the roller tops. | `STOP_X`, `STOP_UP` |
| Station | The tote's rear face past x 80 (yellow line across the rollers). | `ZONE_REAR` |
| Load | Polypropylene tote 14 x 12 x 6 cm, 350 g, with a 500 mL square HDPE bottle 6 x 6 x 16 cm, 500 g, standing loose in its middle: 850 g. | `TOTE`, `TOTE_KG`, `BOTTLE`, `BOTTLE_KG` |

## Real-rig calibration: the loaded tote's deceleration

The release-speed window depends on how fast the rollers slow the tote. A real roller bed's resistance comes from
bearing and seal drag and from how the tote's floor beds on the rollers, and it depends on the load. So the rig does
not take it from the roller spec: it measures the loaded tote's deceleration and sets it.

**Target and acceptance band:** 0.196 m/s^2, accepted from 0.176 to 0.216 m/s^2 (+/- 10 %, a rolling-resistance
coefficient of 0.018 to 0.022). Across the band, the release-speed window from the task file's model (`window()`)
moves from 0.43-0.61 m/s to 0.47-0.65 m/s and stays about 0.18 m/s wide (0.450-0.629 m/s at 0.196). In the same
model, the demo's 0.50 m/s release still brings the tote to rest at the station.

1. **Light gates.** Fit three through-beam photoelectric gates across the lane, with beams 3 cm above the roller tops
   so they see only the tote's wall. Place them where the tote's front edge crosses x 45, 65 and 85, so the spacing
   is d = 20.0 cm. Set the spacing to +/- 0.5 mm with a steel rule. Log the moment each beam is first broken with a
   timer of 1 ms resolution or better (a photogate timer or a microcontroller). Mount the gates on removable posts
   outside the rails and take them off for trials.
2. **Load.** Use the tote as in the task, with the bottle standing in its middle. The tote meets the end stop below
   0.40 m/s in these runs, so the bottle stays up.
3. **Coast-down.** Push the tote along the lane by hand and let go before its front edge reaches the first gate, so
   it passes the first gate at 0.45 to 0.55 m/s (u1 below). Let it roll on to rest.
4. **Formula.** Let t1, t2 and t3 be the times the front edge breaks each beam. Then:

   ```
   u1 = d / (t2 - t1)        u2 = d / (t3 - t2)        a = 2 (u1 - u2) / (t3 - t1)
   ```

   u1 and u2 are the mean speeds over the two spans. Under a constant deceleration, each one is the speed at the
   middle of its span's time, and those two moments are (t3 - t1) / 2 apart, so the formula is exact. For example,
   at 0.196 m/s^2 and 0.50 m/s on release, t2 - t1 = 0.448 s and t3 - t2 = 0.579 s give u1 = 0.446 m/s,
   u2 = 0.345 m/s and a = 0.197 m/s^2. A 1 ms error at the middle gate moves a by about 1.6 %.
5. **Repeat** five times: two runs with u1 near 0.45 m/s, two near 0.55 m/s and one between. Accept when the mean is
   within 0.176-0.216 m/s^2 and every run is within 0.02 m/s^2 of the mean. If the slow runs and the fast runs differ
   by more than 0.02 m/s^2, the drag depends on speed (a sticking or braked roller): find that roller and replace it.
6. **Setting it.** Do not use speed-controlled brake rollers. Their drag rises with speed, while the sim's
   deceleration is the same at every speed.
   - First spin every roller by hand. Each should turn freely and coast down smoothly. Replace any that drags or
     stops well short of its neighbours.
   - Then set the conveyor's grade, uniform along its whole 93.5 cm frame, by shimming its feet. Each 1 mm of rise
     at the far end adds 9.81 x 0.001 / 0.935 = about 0.0105 m/s^2, and each 1 mm of fall takes the same off.
   - If the tote is too free (below 0.176 m/s^2), raise the far end. If it is too stiff (above 0.216 m/s^2), lower
     it. Then repeat step 5 and record the final grade.
   - If it needs more than 4 mm either way (about 0.04 m/s^2, a slope of 0.25 deg), service the rollers instead of
     tilting further. At that slope, the tote and bottle stay far inside the scorer's 8 and 15 deg limits.
7. **When.** Calibrate before a session's first trial and after any roller change. Check with one coast-down every 10
   trials; it must fall inside the band.

## Reset between trials

Roll the tote back to the near end of the conveyor (rear face 17.5 cm ahead of the arm bases) and stand the bottle upright in its middle.
