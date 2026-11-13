# Change the blood tube on a needle already in a training arm's vein: fill it and take it off without moving the needle

Instruction given to the policy: `On the practice arm, fill the blood tube, then lay it in the tray.`

Files: `task.json`, `venipuncture_tube_exchange.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`,
two sim controls and an offline self-test), `mechkit.py` (unchanged helper: sliders with friction, per-episode memory),
`README.md`.

## Environment

Isaac Lab-Arena 0.3.1 with yam-arena and inspect-robots, as the program's launchable installs them. Before every shell:
`source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/venipuncture-tube-exchange/venipuncture_tube_exchange.py
```

It checks the geometry (needle tip under the skin, stopper through the bore, the squeeze span inside the gripper's
94 mm, the flange wing where the squeeze expects it), asserts the scoring rules on hand-built states, and replays every
plan (the demo and both sim controls) through the kinematics at 20 Hz, every joint inside the URDF limits, IK error under
0.5 mm. Expected output:

```
scoring: 19 cases pass
demo left: 48.4 s plan, worst IK 0.088 mm / 0.013 deg
demo right: 42.4 s plan, worst IK 0.099 mm / 0.014 deg
control a (not the demo) left: 10.0 s plan, worst IK 0.000 mm / 0.000 deg
control a (not the demo) right: 10.0 s plan, worst IK 0.083 mm / 0.014 deg
controls c+b (not the demo) left: 24.0 s plan, worst IK 0.088 mm / 0.013 deg
controls c+b (not the demo) right: 21.7 s plan, worst IK 0.098 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/venipuncture-tube-exchange/venipuncture_tube_exchange.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "On the practice arm, fill the blood tube, then lay it in the tray." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/venipuncture-tube-exchange-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`), otherwise
`score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/venipuncture-tube-exchange/venipuncture_tube_exchange.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "On the practice arm, fill the blood tube, then lay it in the tray." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/venipuncture-tube-exchange-demo
```

The demo is `demo()` in the task file, 48 s of plan. In the sim run it ended on success at about step 830 (41 s), inside the run limit of 1,800 steps: the stopper pierced under a 4.4 N push, the tube went home 7.5 mm and filled to 100 %, it was drawn off and laid in a tray slot, and the needle stayed within 0.55 mm of its start throughout.

## The mechanism (what is physical, what is a rule)

- **Needle in the arm.** Holder and needle are one rigid body on a slide along the needle's own line, anchored in the
  training arm, with 0.5 N of Coulomb friction (tissue drag; a force-capped joint drive whose stick point follows the
  needle every physics step). The vein's lumen is +-2 mm of that slide. Any push or pull on the holder over 0.5 N moves
  the needle; nothing else holds it.
- **Stopper.** The tube rides a slide joint along the needle's line in the holder's bore (as the bore guides it). Its
  drive is a spring whose anchor stands for friction (mechkit's model): seated at the guide line, the unpierced tube is
  held there; a push that compresses the spring past 4 N (the tube 1.3 mm in) means the stopper has given way, and from
  then on the tube slides on the needle with up to 1.5 N of drag, ramped over the 4 mm of stopper the needle passes
  through, to a stop 8 mm in. Drawn back clear of the needle the drag is the bore's 0.3 N; once the stopper is 2 mm out
  of the holder's mouth the joint is switched off and the tube is free. The anchor moves only while a finger tip
  touches the tube (static friction: with no applied load the tube stays where it is). This is needed in the sim: the
  light holder and tube carry small joint-error forces when a gripper squeezes the holder, which otherwise walk the
  tube along its slide. The drive's force caps and anchor are updated once per control step (20 Hz).
- **Fill.** A counter, not a fluid: it runs (4 mL in 8 s) only while the needle tip is through the stopper and the
  needle is in the vein. The tube's body turns red in fifths as it fills (looks only). On a real training arm the
  artificial blood flows only from the vein, so the fill line checks the needle position as well.
- **Holding the holder.** Read from contacts: a gripper holds the holder when both its finger tips touch it, or one
  touches it and the other the tube (the thumb-and-fingers squeeze of flange and tube end).

Forces are estimates, not measurements: tissue drag 0.5 N, bore drag 0.3 N, stopper piercing 4 N (ISO 8871-5 caps the piercing force of
vial closures at 10 N; no figure for blood-tube stoppers was found), stopper drag 1.5 N, fill 8 s for 4 mL.

## Scoring rules

- Latched 0: the needle ever moved more than 2 mm along its line (either way); the holder let go of after it was first
  held (no hand on it for 0.25 s); the stopper never pierced.
- 0.25 pierced with the needle in the vein; 0.5 filled to the mark (>= 90 %); 0.75 that and the tube off the needle and
  out of the holder; 1.0 that and the tube lying in a tray slot, let go and still, the holder still held (held 1 s).

## Sim controls

The task file has two control plans besides `demo()` (each was run as `demo` in a debug copy, no frames):

- `controls_push()` (a): nothing holds the holder; the right hand's closed finger tips push the tube in by its end.
  Logged: `[vp] LATCH needle-out: the needle moved +2.02 mm along its line (deeper, through the vein; lumen +-2 mm) at
  physics step 1325, waypoint '[ctl-a] right: push the tube in, holder free'`, task_score 0.
- `controls_letgo_yank()` (c, then b): as the demo up to the tube grip, then the left hand lets go of the holder and the
  right hand yanks the tube out. Logged: `[vp] LATCH holder-let-go: neither hand on the holder for 5 control steps, at
  physics step 4656 (needle +3.19 mm), waypoint 'right: hold the tube (left lets go)'`, task_score 0. In that run the
  needle had already gone deeper (+2.28 mm) as the left hand released, before the yank; the yank itself moved it only
  0.4 mm (it cleared the stopper in about three control steps, faster than the 20 Hz friction update).

## How the demo does it

The left gripper holds the holder's barrel near the hub, closing across it. The right gripper does the push the way a
phlebotomist's thumb and fingers do: one pad behind the flange's upper wing, the other on the tube's end, its closing
axis tipped 13 deg off the needle's line so the flange pad clears the barrel. Closing it pushes the tube onto the needle
with no net force on the holder. It then lets go, grips the tube's body, waits for the fill, eases the tube off (2 mm/s
over the stopper, then 10 mm/s), turns it level and lays it in a tray slot. The scripted demo reads sim state, as the
Guide allows: the left hand's target is corrected every control step against the needle's measured travel, so it keeps
the holder still while the tube's drag pulls on it.

## Dream-1 run

One Dream-1 trial (1,800 steps, 90 s): task_score 0. It moved toward the arm but never pierced the stopper.

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz). The demo plan is 48 s.
- One fixed layout: `-E object_jitter=0`. Dream-1 is hosted and not seeded; the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up.

## Reset between trials

Set the needle back to its start mark (9 mm in), replace the tube with a fresh evacuated one (a filled or pierced tube
has no vacuum left) pushed into the holder to the guide line only, and empty the tray. The needle is sharp: keep people
clear of the training arm while the robot works.

## Assets

All geometry is built from boxes in the task file (no third-party meshes). `mechkit.py` is this repository's helper.
