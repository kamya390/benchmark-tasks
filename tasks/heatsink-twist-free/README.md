# Take a heatsink off a processor without pulling the processor out of its socket

Instruction given to the policy: `Take the heatsink off the processor and put it on the grey mat. The processor stays in its socket.`

Files: `task.json`, `heatsink_twist_free.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `mechkit.py` (helper module imported by the task file: the processor's and the heatsink's breakable bonds;
it patches yam_arena in memory at import, nothing on disk changes; written for this task set, no third-party code),
`README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/heatsink-twist-free/heatsink_twist_free.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 22 cases pass
left: 21.4 s plan, worst IK 0.098 mm / 0.015 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/heatsink-twist-free/heatsink_twist_free.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the heatsink off the processor and put it on the grey mat. The processor stays in its socket." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/heatsink-twist-free-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/heatsink-twist-free/heatsink_twist_free.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the heatsink off the processor and put it on the grey mat. The processor stays in its socket." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/heatsink-twist-free-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 21 s of plan (about 628 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/heatsink-twist-free-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/heatsink-twist-free-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

The board, socket and mat never move. Each trial gets a fresh compound film.

**Sim:** the processor is re-seated and both bonds are re-made, with the same numbers as `heatsink_twist_free.py`:

| Bond | Code | Value |
|---|---|---|
| Socket hold on the processor | `CPU_HOLD_N` | breaks at 6 N |
| Compound, straight pull | `PASTE_PULL_N` | tears at 25 N |
| Compound, twist | `PASTE_TWIST_K`, `PASTE_RELEASE_DEG` | a 2 N*m/rad spring that lets go past 4 deg (2 x 0.070 rad = 0.14 N*m) |

The socket walls (a 41 mm opening round the 40 mm processor) stop it turning.

### Real rig

**Processor and socket retention.**
- The processor is a dummy coupon: 40 x 40 x 4 mm FR-4 under a nickel-plated copper lid, 30 g, with a 0.5 mm
  mild-steel disc bonded under it.
- A small neodymium disc magnet sits in a hole in the board under the socket's centre, and holds the coupon down.
- Non-magnetic spacers under the magnet set the coupon's pull-off to 6 N. The walls stop it turning, as in sim.

**Compound.** A non-curing, silicone-based, metal-oxide-filled thermal grease (the general-purpose heatsink compound
category). It does not cure, so there is no curing step; the bond line and the settle time are what is controlled.
1. Wipe the lid and the heatsink's base with isopropyl alcohol and lint-free wipes, then let them dry 2 min.
2. Frame the lid's edge with tape about 0.1 mm thick (measure it).
3. Put 0.5 g of grease on the lid, weighed on a 0.01 g scale. Spread it across the frame with a flat plastic card,
   in one pass each way, to a uniform film over the whole lid (about 0.4 g stays). Peel the frame.
4. Set the heatsink square on the lid, fins along x. Press it down with a 2 kg mass for 60 s, without twisting.
5. Remove the mass and let it settle 10 min at 20-25 C. Use the film within 2 h.

## Real-rig calibration

These are acceptance checks: a rig that fails one is re-tuned as described, not used.

**Tools.**
- Spring scales: 0-10 N (0.1 N divisions) and 0-50 N (0.5 N divisions).
- A dial torque gauge, 0.02-0.5 N*m with a peak-hold pointer.
- A protractor ring on the board round the socket.
- An adapter: a bar clamped across the fin tops, with an eye and a hex boss on the heatsink's vertical axis. Weigh
  it and subtract its weight from every reading below.

**When.**
- Check 1: at the start of each session.
- Checks 2 and 4: on the first trial of each session and after any change of grease batch.
- Check 3: once per grease batch.

Pull straight up, smoothly, at about 10 N/s. A grease film's pull-off depends on the pull rate and the bond line, so
both are fixed.

1. **Socket hold.** With no heatsink, pull the coupon straight up with the 0-10 N scale (a small suction cup on the
   lid). It must lift at 6.3 +/- 0.3 N (the 6 N hold plus its 0.3 N weight), 3 times in 3. Adjust the spacers until
   it does.
2. **The hold is weaker than a straight pull on the bond.** Make a fresh bond in the socket and pull the heatsink
   straight up by the adapter's eye. The coupon must come out stuck to the heatsink, the scale peaking near 8.7 N
   (6 N + 2.45 N heatsink + 0.3 N coupon), 5 times in 5. This is the failure the task scores 0.
3. **Bond strength.** Bond a second coupon screwed to a test plate, so it cannot let go, and pull the same way with
   the 0-50 N scale. The film must hold at least 15 N above the heatsink's 2.45 N weight; the target is 25 N, the sim
   value (a 27.5 N reading). If it parts lower, use a thinner frame or a higher-viscosity grade.
4. **Twist release.** Make a fresh bond in the socket. Turn the heatsink about the vertical with the torque gauge on
   the hex boss, slowly (about 10 deg in 3 s), on to 20 deg (the demo's twist). The peak torque must be
   0.14 +/- 0.04 N*m and come within the first 10 deg. In sim it rises to 0.14 N*m at 4 deg, then lets go. Then lift
   with the 0-10 N scale. The heatsink must come off at under 4 N (its 2.45 N weight plus under 1.5 N of film), and
   the coupon must stay seated, 5 times in 5.

Checks 2 and 4 together are the property the task rests on: a straight pull takes the processor with the heatsink,
and a twist of about 0.14 N*m frees the heatsink first.
