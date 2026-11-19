# Open a new superglue tube: unscrew the cap, flip it and pierce the foil seal with its pin

A superglue-style tube stands in a fixed bench holder. The cap is turned 1.5 turns counter-clockwise (M14 x
1.5 mm), turned over, and its crown pin is driven squarely into the foil that seals the nozzle, then withdrawn and
released, all without pressing on the soft tube body.

## Run

```sh
inspect-robots run --embodiment yam_arena -E task=pierce_glue_tube_foil_seal.py \
  --instruction "Unscrew the glue tube's cap, flip it over, and use the pin on top of the cap to pierce the foil seal on the nozzle without squeezing the tube. Leave the cap and tube released." \
  --scorer task_score
```

Offline checks (no simulator): `python3 -m unittest test_scoring.py`

Run limit 3000 steps (150 s at 20 Hz). Seeds: none; the layout is fixed.

## Layout

- `pierce_glue_tube_foil_seal.py`: scene, thread/foil joints, per-step pose feed, `score()`, `success()`, `demo()`.
- `scoring.py`: pure tracker and scoring ladder (0, 0.25, 0.5, 0.75, 1.0) with whole-episode failure latches.
- `test_scoring.py`: untouched, body squeeze, off-axis shear, foil tamper, dropped cap, axial pull, clockwise turning,
  each tier, and each full-score condition (retraction, release, 1 s stillness).
- `bodykit.py`: vendored composite-body/joint helper.

## Scoring (also in task.json)

Latched for the episode, any one zeroes the score: a finger pad or TCP within 5 mm of the soft body; a pin
engaged on the nozzle at more than 10 degrees tilt; foil displaced 2 mm without an aligned pin contact; cap
below the bench. The thread releases only after 530+ degrees of counter-clockwise turning, so pulling or clockwise
turning cannot detach the cap. Full score needs both grippers 4 cm from cap and nozzle and 1 s of rest.

## Validation

The scripted `demo()` reaches 1.0 in YAM Arena (Isaac Sim 6.1, 20 Hz, seed 0, no jitter) in one episode: four
counter-clockwise wrist strokes free the cap (about 530 degrees of turning), it is tipped onto its side and set down,
re-gripped from above and tilted pin-down, the pin is driven through the foil and withdrawn, and the cap is laid on
its side on the bench with the gripper clear. The foil's 3 N break force, the thread release and the squeeze margin
are uncalibrated estimates; a real tube should be used to measure them.

## Assets and licence

All geometry is procedural primitives built in this folder (`bodykit.py` is our own helper code). No third-party meshes, textures, scans or other assets are used.
