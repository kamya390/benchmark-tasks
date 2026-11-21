# Turn a cake out of its tin: plate on top, flip both together, lift the tin off

A 20 cm cake tin with a snug 0.5 kg cake stands on a post; a flat 25 cm plate lies on a second post. The robot lays
the plate face-down on the tin, clamps plate and tin together with both grippers, turns the pair over in one
synchronized roll of both wrists, sets it down, and lifts the tin straight off, leaving the cake centred on the plate.

## What it tests

A clamped two-body inversion with a loose payload. Plate and tin are two separate rigid bodies held face to face only
by the two grippers; the cake is held by neither gripper. Part-way through the turn the cake lets go of the tin and is
then contained only by the tin wall and the plate. If a clamp opens a gap, or the two wrists turn out of step, the
cake slides out sideways and the episode scores 0. The tin then has to come off without dragging the cake: it has no
handle, so it is lifted by pressing on its two sides.

It differs from flipping one hinged appliance (the two halves there are joined), from pushing a tart up out of a ring,
from lifting a slab by a sling, and from opening a springform clasp: here nothing joins the three bodies.

## Run

```sh
inspect-robots run --embodiment yam_arena -E task=turn_cake_out_of_tin.py \
  --instruction "Turn the cake out of its tin onto the plate." --scorer task_score --max-steps 2400
```

Scripted demo: add `--policy yam_scripted -E object_jitter=0 -E control_hz=20`. Offline scorer tests (no simulator):
`python3 -m unittest test_scoring.py`. Run limit 2,400 steps (2 minutes at 20 Hz); the demo takes about 1,330 (66.6 s). No
seeds: the layout is fixed.

## Setup

| Object | Size, material, mass | Start |
|---|---|---|
| Two stands | 110 mm diameter, 140 mm tall, rigid, fixed to the bench | bench centre line, 450 mm and 200 mm in front of the robot bases |
| Cake tin | 20 cm round, fixed base, aluminium; 200 mm inside at the rim, 195 mm at the floor, 50 mm deep, rolled rim; 0.25 kg | open side up on the far stand |
| Cake (inert stand-in) | flexible polyurethane foam (about 0.4 g/cm3) cast in that tin on a parchment disc, trimmed to 44 mm; 0.50 kg | in the tin, stuck lightly with a pea of reusable poster putty |
| Plate | flat, round, 250 mm across, 8 mm thick, 0.28 kg (e.g. a wooden pizza/cake board); ring drawn on the eating face | eating face up on the near stand |

The stands give the gripper fingers room to pass under the tin and the plate, as a counter edge does at home; they do
not hold, guide or turn anything.

## Score (also in task.json)

| Score | Condition |
|---|---|
| 0.25 | plate face-down on the upright tin's rim (within 5 mm), centre within 2 cm of the tin's axis, cake still in the tin |
| 0.5 | plate and tin stayed together throughout inversion; pair now at rest, plate underneath, level, face-up and supported on the bench or a stand, tin upside-down over the cake, cake resting on the plate |
| 0.75 | at the end: tin clear of the cake (2 cm above or 2 cm apart) and the cake resting on the set-down plate |
| 1.0 | also: cake centre within 2 cm of the plate centre, cake flat within 5 degrees with its parchment base up, tin resting on the free stand with its centre within 3 cm of the stand's (either way up; upside-down it sits over the stand like a cap), every finger 3 cm from tin, plate and cake, all still for 1 s |
| 0 | the cake ever touches the bench top, or anything falls off the bench |

The first two rows are latched milestones. The inversion milestone requires an observed continuous turn
from the covered upright tin through a sideways orientation. At every 20 Hz frame the plate's eating face
must stay within 5 mm of the tin-rim plane, its centre within 20 mm of the tin axis, and the normals
opposite within 10 degrees. Axis changes above 15 degrees between frames invalidate that attempt.
A broken attempt retains 0.25 and can retry by covering the upright tin again.

At the 0.5 milestone, the cake-to-plate surface gap must be -2 to +5 mm, its axis parallel within 10 degrees,
and its footprint inside the plate. The plate's underside must be within -2 to +5 mm of the bench or a
140 mm stand top, with the whole 55 mm radius stand top beneath the plate's footprint. These tests reject
an unsupported pair. The remaining tiers use the final state. `scoring.py` holds the tracker and ladder.
`test_scoring.py` checks every tier, every full-score condition, both failure latches and continuous-turn shortcuts.

## Fidelity

Rigid-body Isaac Sim/PhysX at 20 Hz. Everything is full size and full mass. The tin is a floor disc with 24 tapered
wall panels (2 mm thick; the rolled rim is drawn but does not collide); the cake is three stacked discs following the
taper with 1.5 mm of clearance. The cake does not deform or crumble. Its sticking to the tin is a fixed joint that
the task releases for good once the cake's own weight pulls on it along the tin's axis with more than 3 N (tin tipped
past about 128 degrees from upright). Until then the cake cannot move in the tin; after that it is a loose body. The
3 N value and the friction coefficients are estimates.

## Validation

A complete replay from the standard start scored **1.0 under the revised scorer** at step 1320 of the 2400-step
limit (66 seconds, Isaac Sim, 20 Hz, no object jitter). Its log ends with `task_score: 1` and `DONE rc=0`. The command requested CLI seed 0;
the Isaac environment log reports seed 42.

| Replay milestone | First logged step |
|---|---|
| Plate placed on upright tin, 0.25 | 440 |
| Continuous inversion and supported set-down, 0.5 | 880 |
| Tin clear, cake on supported plate, 0.75 | 1220 |
| Full released and settled result, 1.0 | 1320 |

The existing demo video shows the original recorded trial, which scored 1.0 under the original scorer.
The revised-scorer replay used the same scene geometry, mass, scripted motion and cake-release rule.
It stored no frames. The video supplies visual evidence of the motion, while the replay log establishes
that the complete motion also passes the revised sequence and support checks. These are separate trials.
The replay's cake finished 6.9 mm from the plate centre.

The existing Dream-1 video scored 0.0 under the original scorer and shows the scene untouched. An untouched
scene also scores 0.0 under the revised scorer, covered by the untouched-scene regression test. Dream-1
was not rerun for this revision. The original recording command gave Dream-1 the instruction
"Turn the cake out of its tin onto the plate." This matches `task.json`, `INSTRUCTION` and the run command.
The scripted validation replay received the equivalent expanded wording "Lay the plate face-down on the
tin, turn the two over together, set them down and lift the tin off, leaving the cake on the plate."
The scripted policy follows `demo()` independently of instruction wording.

Both grippers pinch the plate's edge, roll it face-down onto the tin, clamp the pair, turn it over together,
set it down and squeeze the tin's sides to lift it off. The squeeze uses the arms' compliance, with about
40 mm of commanded overlap at the far stand. Finger friction is the simulator default. Physical transfer
of this grip force has not been measured.

`CAKE_STAGE=clamp` and `CAKE_STAGE=lift` are debugging starts. The lift start seeds the earlier milestones
to isolate tin removal. Both recorded media and the full validation replay use the standard start with
`CAKE_STAGE` unset. Only that start supplies full-task evidence.

## Real trial and reset

- Cast the stand-in once: line the tin floor with a parchment disc, pour flexible polyurethane casting foam of about
  0.4 g/cm3 (25 lb/ft3), let it cure, trim the top to 44 mm and check 0.50 kg (+-0.03) on a kitchen scale. It then has
  the tin's own taper. It is dry and inert, so the grippers stay clean.
- Sticking: a pea of reusable poster putty between the parchment and the tin floor. Check by hand before a session:
  the cake stays in with the tin on its side and drops out within 3 s with the tin upside-down. Add or remove putty
  until it does.
- Reset: cake back in the tin parchment-side down and pressed onto the putty; tin on the far stand; plate face-up on
  the near stand; both centred on pencil rings drawn on the stand tops. Nothing is consumed.
- Record fixed front and side phone videos, preferably at 60 fps. Put millimetre scale cards next to the rim
  and a printed 10-degree angle guide behind the rotation plane. Keep the supports and both plate edges visible.
  Inspect the footage frame by frame to verify that the plate stays against the tin while both turn together
  through the sideways pose. A gap above 5 mm, centre offset above 20 mm or relative-axis error above 10 degrees
  breaks that attempt. Covering the upright tin again allows a retry. A robot turning only the tin onto a
  stationary face-up plate cannot earn 0.5.
- At set-down, use the side view and a ruler to verify plate contact with the bench or stand within 5 mm,
  the stand top fully beneath the plate, and the cake resting on the plate within the same gap tolerance.
  The simulator permits 2 mm of numerical interpenetration at these contacts.
- Use a ruler for rim-to-rim gaps on four sides (centre offsets), a phone inclinometer on the supported plate
  and cake (level and flatness), and both videos for release, bench contact and the one-second stillness check.
  The simulator's 15-degree per-step axis-jump guard detects a discontinuous sampled trajectory. In a real
  trial the continuous video supplies the corresponding continuity evidence.

## Layout

- `turn_cake_out_of_tin.py`: scene, per-step pose feed, `score()`, `success()`, `demo()`.
- `scoring.py`: pure tracker and ladder. `test_scoring.py`: offline tests. `bodykit.py`: our composite-body helper.

## Assets and licence

All geometry is procedural primitives built in this folder (`bodykit.py` is our own helper code). No third-party
meshes, textures, scans or other assets are used.

## Response to review of TK-SWG72E

The tracker now follows uninterrupted plate-to-tin contact through inversion and permits a fresh attempt
after separation. It verifies support beneath the plate and cake-to-plate contact before granting 0.5.
Regression controls cover a stationary plate, detached intermediate plate, endpoint-only pose jump,
unsupported low pair, missing stand, hovering cake, a successful retry, and reset between episodes.
The revised scorer passes 32 offline tests. The stationary-plate control moves the plate away smoothly,
sets it down face-up, then turns only the tin over in small angular increments and lowers it onto the
stationary plate. Each angular increment is below the continuity guard, isolating the missing joint turn.
That sequence remains at 0.25. A valid continuous paired turn followed by release scores 1.0, including
a successful retry after a broken attempt. The full simulation replay also scored 1.0 at step 1320.
