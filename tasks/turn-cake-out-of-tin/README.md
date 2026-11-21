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
| 0.5 | then: pair turned over and at rest, plate underneath, level, face-up and set down (underside at most 15 cm above the bench), tin upside-down over the cake, cake on the plate |
| 0.75 | at the end: tin clear of the cake (2 cm above or 2 cm apart) and the cake resting on the set-down plate |
| 1.0 | also: cake centre within 2 cm of the plate centre, cake flat within 5 degrees with its parchment base up, tin resting on the free stand with its centre within 3 cm of the stand's (either way up; upside-down it sits over the stand like a cap), every finger 3 cm from tin, plate and cake, all still for 1 s |
| 0 | the cake ever touches the bench top, or anything falls off the bench |

The first two rows are latched milestones; the rest is read from the final state. `scoring.py` holds all of it and
`test_scoring.py` checks every tier, every full-score condition and both failure latches.

## Fidelity

Rigid-body Isaac Sim/PhysX at 20 Hz. Everything is full size and full mass. The tin is a floor disc with 24 tapered
wall panels (2 mm thick; the rolled rim is drawn but does not collide); the cake is three stacked discs following the
taper with 1.5 mm of clearance. The cake does not deform or crumble. Its sticking to the tin is a fixed joint that
the task releases for good once the cake's own weight pulls on it along the tin's axis with more than 3 N (tin tipped
past about 128 degrees from upright). Until then the cake cannot move in the tin; after that it is a loose body. The
3 N value and the friction coefficients are estimates.

## Validation

The scripted `demo()` reaches 1.0 in YAM Arena (Isaac Sim, 20 Hz, seed 0, no jitter) in about 1,330 steps. Both
grippers stay horizontal and face each other the whole time: they pinch the plate's edge on both sides and roll it
face-down onto the tin; clamp plate and tin with one finger over the plate and one under the tin, carry the pair
between the stands and roll both wrists half a turn in step (the cake lets go of the tin about two-thirds of the way
round and settles on the plate); set the pair down; then press two fingertips on each side of the tin, lift it off
the cake and lower it over the free stand. The squeeze uses the arms' own compliance (about 40 mm of commanded
overlap at the far stand); the finger friction is the simulator default. In the demo the cake ends 7 mm from the
plate's centre.

## Real trial and reset

- Cast the stand-in once: line the tin floor with a parchment disc, pour flexible polyurethane casting foam of about
  0.4 g/cm3 (25 lb/ft3), let it cure, trim the top to 44 mm and check 0.50 kg (+-0.03) on a kitchen scale. It then has
  the tin's own taper. It is dry and inert, so the grippers stay clean.
- Sticking: a pea of reusable poster putty between the parchment and the tin floor. Check by hand before a session:
  the cake stays in with the tin on its side and drops out within 3 s with the tin upside-down. Add or remove putty
  until it does.
- Reset: cake back in the tin parchment-side down and pressed onto the putty; tin on the far stand; plate face-up on
  the near stand; both centred on pencil rings drawn on the stand tops. Nothing is consumed.
- Checks: ruler for the rim-to-rim gaps on four sides (centre offsets), a phone inclinometer app on the plate and the
  cake (level, flat), two fixed phone videos (front and side) for the milestones, the bench-touch rule and stillness.

## Layout

- `turn_cake_out_of_tin.py`: scene, per-step pose feed, `score()`, `success()`, `demo()`.
- `scoring.py`: pure tracker and ladder. `test_scoring.py`: offline tests. `bodykit.py`: our composite-body helper.

## Assets and licence

All geometry is procedural primitives built in this folder (`bodykit.py` is our own helper code). No third-party
meshes, textures, scans or other assets are used.
