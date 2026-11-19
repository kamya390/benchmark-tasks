# Clean an earbud and replace its eartip

This is the first build of the investigated Samsung Galaxy Buds+ service task. It intentionally covers the full
procedure: remove and discard the old tip, use the dry brush on the speaker mesh, use the cotton swab on the charging
contacts, seat a fresh tip, and release the work.

## What is implemented

- A 6.3 g earbud with a 22.5 x 17.5 x 19.2 mm compound-primitive envelope, held still in a low silicone service cradle.
- Old and replacement silicone tips with a provisional compliant capture/seat mechanism.
- Separate dry brush and reusable electronics swab. The brush rests on a tray ledge under a light clip that lets go once it is gripped; the fresh tip stands on a presentation peg.
- Cleaning credit that requires the correct tool, surface contact, and accumulated tangential travel. Proximity and
  wrong-tool contact do not count. Individual brown inspection specks disappear as stroke quanta are completed.
- Live disposal, replacement-seat, release, and rest checks; irreversible cleaning history; an offline-tested score
  policy in `scoring.py`.

Run the simulator task from this directory with the repository's normal YAM Arena environment:

```sh
inspect-robots run --embodiment yam_arena -E task=clean_earbud_and_replace_eartip.py \
  --instruction "Remove the worn eartip and put it in the discard cup. Dry-brush the speaker mesh, clean the charging contacts with the cotton swab, then fit the new eartip. Leave the earbud and tools released." \
  --scorer task_score
```

Run the simulator-independent checks with:

```sh
python3 -m unittest test_scoring.py
```

## Validation

The scripted `demo()` reaches 1.0 in YAM Arena (Isaac Sim 6.1, 20 Hz, seed 0, no jitter) in one uninterrupted
episode: worn tip pulled and dropped in the cup, all eight mesh specks brushed off, all six contact specks swabbed
off, fresh tip carried, caught and seated, both tools back on their marks and the arm parked clear. Dream-1 scores
0.0. Silicone resistance, the capture cone, seated gap and contact bands remain uncalibrated against a physical
earbud; a real Buds+ or dimensionally representative mock should measure them.

## Source and audit notes

- Samsung's official cleaning and eartip procedure: <https://www.samsung.com/us/support/answer/ANS10001320/>
- The fresh board audit found no exact task among 719 entries. Closest accepted mechanics were O-ring seating (#1576),
  electric-toothbrush head replacement (#1621), fibre-connector cleaning (#1925), and scrub-the-hub (#1013).
- The charging case is excluded to stay distinct from pending AirPods case-packing PR #2784 and rejected earbud-case
  snap task #1247.
