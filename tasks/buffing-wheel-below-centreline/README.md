# Polish both faces of a blank on a running buffing wheel without letting it snatch

Instruction given to the policy: `Polish both faces of the aluminium blank on the buffing wheel, then stand it back in its rack.`

Files: `task.json`, `buffing_wheel_below_centreline.py` (scene, blank/wheel contact monitor, `success()`, partial-credit
`score()`, the scripted `demo()` and an offline self-test), `mechkit.py` (helper module imported by the task file: the
two wheels' spindle joints and the per-episode hooks; it patches yam_arena in memory at import, nothing on disk
changes; written for this task set, no third-party code), `README.md`.

## How polishing is measured

- **Real contact only.** The blank carries PhysX's contact-report API and a rigid-contact view (the mechanism behind
  Isaac Lab's ContactSensor) pairs it with the two wheels. It is read after every physics step: the contact points
  with their separations, and the contact force between the blank and each wheel (the view's force matrix, Isaac
  Lab's `force_matrix_w`). A step polishes a face only when a wheel touches it (points at 0.5 mm separation or less,
  on the face, on the wheel's down-running front below the spindle) and presses it (at least 1 N along the face's
  normal; the wheel's drag lies in the face's plane). PhysX also lists points inside its contact offset (a face
  hovering a millimetre or so off the rim), but they are not touching and carry no force: they earn nothing. Contact
  on the blank's edges earns nothing.
- **Coverage, not dwell.** Each face is divided into ten 10 mm strips along its length. While a face is pressed, the
  strips under the wheel's 25 mm width get the step's time (the rigid 36-sided buff often touches a slightly skewed
  face at one rim edge, where a cloth buff bears on its whole width). A strip is polished after 0.5 s; a face is
  polished with 8 of its 10 strips. A face held still in one place covers at most 3 strips: the blank has to be worked
  across the wheel, and the end held in the fingers needs a change of grip.
- **Snatch and fling latch** for the whole trial (0 even if the blank is put right afterwards).
- On a real rig, the same rule can be read from a wrist force sensor (press) plus the trial video (where the wheel
  ran), or checked after the trial with a gloss meter (60 deg geometry, ISO 2813), one reading per strip.

## Real-world rig: guarding and protection

The sim's wheels turn at 2 rev/s; a real bench buffer runs at about 1,750 or 3,450 rpm, coasts for several seconds
after switch-off, and throws a snatched part hard. A lab reproducing this task should:

- **Guarded cell.** Run the buffer only inside the robot cell's fixed perimeter guard (robot-system safeguarding per
  ISO 10218-2; guards per ISO 14120; safety distances per ISO 13857). The door interlock cuts both the robot drives
  and the buffer's motor contactor, with guard locking (ISO 14119) held until the wheel has stopped.
- **Wheel hood and catch screen.** A hood over each wheel exposing only its lower front working quadrant; a fixed
  polycarbonate screen in front of and below the work rests and a tray under them, so that a thrown blank stays in
  the cell. Keep the work-rest gap at 3 mm or less, as in the scene.
- **Speed and force.** First trials on a variable-speed buffer at its lowest speed. Use an arm with power and force
  limiting (ISO/TS 15066) or a wrist force sensor with a stop limit set above the task's press (about 17 N mean, 45 N peak in sim), so that
  a finger jammed against the wheel stops the arm.
- **Emergency stop** (ISO 13850) outside the cell within the operator's reach, wired to both the robot and the buffer.
- **People.** Nobody in the cell while the buffer runs. Anyone loading or inspecting parts wears safety glasses with
  side shields or a face shield (ANSI Z87.1 or EN 166). No gloves, loose sleeves, jewellery or loose long hair near a
  running wheel. Use hearing protection if the measured noise calls for it. Put local exhaust at the wheel for lint
  and compound dust.
- **Verification a lab can run before trials:**
  1. With the buffer running, open the cell door. Check that the arm stops and the motor power drops. Time the wheel's
     run-down with a hand tachometer, and check that the guard lock stays engaged until the wheel reads 0 rpm.
  2. Repeat with the emergency stop from the operator's position.
  3. Throw test: with the cell closed, run the naive presentation (the blank held above the centreline). Confirm that
     the blank ends inside the hood, screen and tray.
  4. Drive the empty gripper slowly against a load cell placed on the work rest. Confirm that the arm stops at its set
     force limit.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/buffing-wheel-below-centreline/buffing_wheel_below_centreline.py
```

It asserts the scoring rules on hand-built states and contact reports (including a hover, a graze, a face held still in one
place, contact above the centreline, on the wheel's back or on an edge, rushed presses and 7 vs 8 strips), and replays every arm's demo plan through the
kinematics at 20 Hz (every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 23 cases pass
right: 103.5 s plan, worst IK 0.097 mm / 0.012 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/buffing-wheel-below-centreline/buffing_wheel_below_centreline.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Polish both faces of the aluminium blank on the buffing wheel, then stand it back in its rack." \
  --scorer task_score --max-steps 2800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/buffing-wheel-below-centreline-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`. The run's stdout also prints a `[buff]`
line every 2 s with each face's strip times and the contact counts.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/buffing-wheel-below-centreline/buffing_wheel_below_centreline.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Polish both faces of the aluminium blank on the buffing wheel, then stand it back in its rack." \
  --scorer task_score --max-steps 2800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/buffing-wheel-below-centreline-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 103.5 s of plan (about 2,120 steps including
the success hold), inside the run limit of 2,800 steps. It picks the blank by one end, sets its edge on the right
wheel's work rest and presses its face onto the wheel 25 deg below the centreline, then works it across the wheel in
overlapping presses from the free end toward the middle, easing off between presses to slide along the rest (under
the press, the rest's friction holds the blank); it turns the blank in the hand away from the wheel and does the
other face the same way. It then stands the blank in the rack, re-grips it by the other end, repeats both faces (now the
other half of each), and stands it back in the rack.

## Videos and screenshot

```bash
for log in ~/yam-eval/buffing-wheel-below-centreline-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/buffing-wheel-below-centreline-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 2,800 steps (140 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1 (about 7 minutes for the
  full 2,800), plus Isaac Sim warm-up on the first run.

## Reset between trials

Stand the blank back in its rack, face B away from the buffer; the wheels keep running (on a real rig: stop the
buffer and wait for the guard lock to release before entering the cell, then restart it from outside); the buffer,
rests and rack do not move.
