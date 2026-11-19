# Seat a click-type torque wrench on a busbar lug bolt and torque it to the click

Instruction given to the policy: `Pick up the torque wrench standing on its rest, seat its socket fully on the lug bolt, and torque the bolt with it. The wrench is already set to the bolt's torque. Then let go.`

Files: `task.json`, `click_wrench_stop_at_click.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `mechkit.py` (helper), `README.md`.

## Response to review

**TK-36Y8Z5 — make the pre-seat failure observable.** The old gate inferred bolt rotation from a marker on the
wrench head while the socket was unseated; that inference has been removed. The revised gate measures the marker
itself: after the socket first overlaps the head by 1.0 mm (axis within 12 degrees and 3 mm), the wrench head may
not rotate 1.0 degree before the full 2.8 mm / 8 degree / 1 mm seat has held for five frames. Synchronized side
and top video show the partial/full engagement and the wrench-head marker independently of the bolt. The simulator
uses those same wrench-head and socket poses. A half-seated control now holds the bolt fixed while rotating only
the wrench head and must latch 0, so the test no longer relies on observing bolt motion through an unseated tool.

**TK-VCSWCB — "Robot should be responsible for seating the torque wrench."** Done; the wrench no longer starts
on the bolt. It stands socket-down on the bench 8.5 cm from the joint, its grip on a rest block, and the robot
does the whole job: both hands lift the wrench off its rest (the left at the neck, the right on the grip),
carry it level over the joint and seat the socket on the bolt head; the left then presses the head down, the
two-hand technique of the trade, while the right pulls the grip to the click as before. The seating is physical, not scripted bookkeeping: the bolt is now its
own free body, driven only through a real hex contact (ISO 4017 M5 head, 7.88 mm across flats, chamfered crown,
inside an 8.2 mm 6-point socket cavity built from convex meshes, 0.16 mm clearance per flat, 0.8 mm 45-degree
mouth chamfer, steel-on-steel friction 0.3 per part). The scorer gates on it: "seated" is measured from the two
bodies' poses (at least 2.8 mm of the head's 3.5 mm engaged, within 8 degrees of square and 1 mm of coaxial,
held 5 control steps), the click only counts if the socket is seated when the detent breaks over, and **any
wrench-head rotation of 1 degree or more after partial engagement but before that seat latch is a latched 0**.
A control run ("half" variant: the hands set the
socket down 2.5 mm off the bolt's axis, cocked on the head and never seated, and the right pulls anyway)
scores 0 with its clicks uncredited; the PRESEAT-TURN and LOOSENED latches themselves are asserted case by
case in the offline self-test. Seating credit alone is 0.1.

**TK-NP6A85 — scoring and specification** (kept from the first revision, unchanged here): full credit now
requires the peak torque inside the joint's documented 90-110 % tolerance — the old 1.0 at nearly twice the
setting is a latched 0 (anything past 110 %), and the "pull on through the click" ablation scores 0. The bolt
(M5 x 0.8 x 16 class 8.8, ISO 4017 head geometry), washer (ISO 7089), lug (16 mm2 tube lug, M5 hole), cable
(8 mm over PVC), busbar (20 x 6 mm copper), wrench (ISO 6789-1 Type II class A, 1/4-inch drive, 2-10 N*m,
with the low-cost batch check below), socket (8 mm 6-point, ISO 2725-1, ISO 1174-1 drive) and the snug and torque-angle
calibration are specified below with how a real trial is verified (low-cost peak-torque adapter plus camera). The demo
releases the wrench, parks both arms over 10 cm clear and holds success within the run limit.

**TK-Y3KR6S — full force and affordable verification.** The simulation now uses the real joint's 3.01 N*m
set torque and 14 N grip load; the validated demo clicks at 4.22 degrees / 93 % and scores 1.0. The unspecified
specialist torque-angle transducer and separate wrench tester are replaced by one named, commercially available
1/4-inch digital peak-torque adapter, the Eclatorq TD2-030CN (or an equivalent 1.5-30 N*m adapter with peak hold).
The cited supplier lists it in stock at INR 16,560 including tax as of 2026-11-18; it has 0.01 N*m resolution,
3 % clockwise accuracy and peak/track modes. It remains in the drive line for trials and is also used to check
the click wrench before a batch. Angle comes from the existing top-camera video: a high-contrast radial marker
on the wrench head is tracked against a fixed marker on the busbar. Thus there is one available low-cost meter,
no angle transducer and no separate bench tester.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Real hardware and calibration

The simulated joint is this real one, at full size (sizes as in `setup`):

- **Joint:** a 20 x 6 mm copper flat bar on two glass-polyester standoff insulators; a tinned-copper tube lug for
  16 mm2 (about 6 AWG) cable with an M5 hole (palm 18 x 12 x 3 mm, barrel 8 mm across), crimped on a flexible copper
  cable, black PVC, 8 mm over the insulation; an M5 x 0.8 x 16 mm class 8.8 zinc-plated steel hex bolt (ISO 4017:
  head nominally 8 mm across flats, 7.85-8.00 mm tolerance band, the sim uses 7.88 mm; head 3.5 mm high with the
  standard chamfered crown) with an ISO 7089 flat steel washer (10 mm OD, 1 mm), through the lug palm and the bar
  into an M5 self-clinching nut pressed into the bar.
- **Specified torque: 3 N*m, marked on the lug and on the panel label** (the label is not modelled). Tolerance
  90-110 % (2.7-3.3 N*m).
- **Wrench:** an adjustable click-type torque wrench, ISO 6789-1 Type II class A (permissible deviation +/-6 % under
  10 N*m), 1/4-inch square drive (ISO 1174-1), 2-10 N*m range, about 26 cm long; the grip's load mark 21.5 cm from
  the drive axis, so 3 N*m takes **14 N at the load mark** (snug 7 N; the tolerance's top 15.3 N). The reverse
  selector is modelled as a flush ring, the low-profile pattern common on quarter-inch click wrenches (ring or
  push-through reverse, no proud paddle on the head's top face); the ratchet internals are not modelled. Before
  a batch, exercise it three times, then make five steady pulls at 3 N*m through the same digital adapter used in
  the trials; replace or professionally calibrate the wrench if any peak lies outside its +/-6 % specification.
- **Socket:** an 8 mm 6-point socket, 1/4-inch drive, ISO 2725-1 dimensions: 25 mm long, 12 mm OD, hex opening
  8.2 mm across flats (0.16 mm clearance per flat on the 7.88 mm head), with the usual ~1 mm 45-degree mouth
  chamfer. The hex fit, both chamfers and the clearances are modelled as real convex meshes; the socket drives
  the bolt only through this contact, so it must be seated to tighten it.
- **Wrench rest:** the wrench starts off the bolt, standing socket-down on the bench 8.5 cm from the joint with
  its rubber grip on a 6.3 cm block (any bench block; only its height matters), handle toward the right arm.
- **Handle rest:** a second block (9.8 x 4 cm, 9.7 cm tall, any rigid block) stands under the handle's pull arc
  beside the joint, its top level with the seated grip's underside: it carries the grip whenever the wrench is
  on the bolt - nobody balances the instrumented wrench on a 3.5 mm bolt head - and the grip rides along it
  through the pull. Its friction acts at the same radius as the pull, so it does not shift the click torque.
- **Available peak-torque adapter (in the scene):** Eclatorq TD2-030CN, or an equivalent low-range digital
  adapter, with 1/4-inch square drive in and out, 1.5-30 N*m range, 0.01 N*m resolution, +/-3 % clockwise
  accuracy and peak-hold mode. Goodwill Enterprises lists the TD2-030CN in stock at INR 16,560 including tax
  (observed 2026-11-18):
  <https://goodwill.in/eclatorq-digital-angle-torque-adaptor-angle-meter-1-4-ad2-030cn-1-5nm-to-30nm.html>.
  Its listed overall length is 67 mm; the simulation models the approximately 26 x 26 x 40 mm instrument body
  between its square-drive interfaces. The simulated 450 g head-end mass is conservative relative to available
  adapters. Seated, the wrench's centre plane is 10.6 cm above the table.
- **Camera angle measurement:** put a high-contrast radial marker on the wrench head and a fixed reference marker
  on the busbar. The existing top camera records their relative yaw throughout the pull; because the seated socket
  physically keys the head to the bolt, maximum head yaw is the bolt's maximum turn past snug. Zero the markers
  after snugging. This supplies the angle channel without buying a torque-angle transducer.
- **Sim drive calibration** (sim only): the pipeline's realised angular stiffness is super-linear in its
  command, so the detent was bracketed directly against the full-force joint on the current Box 1 PhysX build.
  Commands up to 1.20 N*m/rad broke over early, while 1.21 N*m/rad did not break over before the 110 % limit.
  The selected `DETENT_DRIVE_CMD = 1.205` produced one seated click and a 4.22-degree / 93 % peak in the stored-
  frame validation run. The designed detent stiffness, 14 N grip load and scored 1.5-degree physical breakover
  threshold are unchanged; this command only compensates for the backend's realised stiffness mapping.
- **Snug (the zero for every angle):** run the bolt down by hand, then tighten until the digital adapter reads
  1.5 N*m (half the marked torque). Align the wrench-head and busbar camera markers at zero. The sim starts here:
  the bolt turns on at half the set torque.
- **Torque-angle calibration** (the turn-of-nut idea: a defined snug, then an angle): on the actual joint, or a joint
  simulator built from the same bolt lot, washer, lug, bar and nut, tighten from snug slowly and steadily and record
  the digital adapter's torque against the camera-marker angle: the angle at 3 N*m and at 3.3 N*m. On three
  sacrificial bolts carry on until the curve's slope falls (yield).
  - The sim's angles are an estimate from the joint's stiffness, not a measurement. With a nut factor of 0.2 the
    preload is 1 kN per N*m, so snug to set adds 1.5 kN; the bolt (with its head and engaged thread) and the lug-bar
    stack give about 7.5 um per kN, so about 11 um of stretch, about 5 deg on a 0.8 mm pitch (the sim uses 4.9 deg, a
    straight line of 3.27 deg per N*m above snug). On that line 3.3 N*m (110 %) is at 5.9 deg. A class 8.8 M5 starts
    to yield under tightening at about 7 kN of preload (640 MPa on 14.2 mm2, less the share taken by thread torsion),
    about 6.9 N*m or 230 % of the set torque, which the same line puts at 17.7 deg, as in the sim.
- **Angle in a real trial:** track relative yaw of the wrench-head and busbar markers in synchronized top-camera
  video. Before seating this is explicitly wrench motion, not bolt motion. After seating, the keyed hex makes the
  maximum head yaw the bolt's turn past snug; it cross-checks peak torque and measures back-off.
- **Seating before tightening in a real trial:** synchronized side video measures engagement and tilt while the top
  view measures coaxial offset and marker yaw. Start the gate when at least 1.0 mm of the head is inside the socket,
  with tilt at most 12 degrees and coaxial error at most 3 mm. From that frame until the full seat (at least 2.8 mm,
  at most 8 degrees and 1 mm) has held for five 20 Hz frames, 1.0 degree or more of wrench-head marker rotation is
  the latched PRESEAT-TURN fail. Thus the check verifies seating before tightening without claiming that an
  unseated wrench marker reveals bolt motion.
- **Peak torque in a real trial:** the digital adapter stays in the drive line for every trial, and its peak-hold
  reading is the trial's peak torque P: 2.7-3.3 N*m with the click is in tolerance (1.0); over 3.3 N*m, as when the
  hand pulls on through the click, scores 0. The video shows the click (the handle's kink) and supplies the angle.
- **Force scale:** the sim uses the real joint's full 3 N*m set torque and approximately 14 N at the grip.
  The validated demo seated to 3.72 mm engagement, clicked once while seated at 4.22 degrees / 93 % torque,
  released cleanly and scored 1.0. Sizes, angles and ratios (snug 50 %, tolerance +/-10 %, yield 230 %) are
  also the real joint's. The sim's bolt body is 30 g (the real bolt and washer are about 8 g) for solver
  stability; the bolt's dynamics are set by its thread-friction hinge, not its mass.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/click-wrench-stop-at-click/click_wrench_stop_at_click.py
```

It asserts the scoring rules on hand-built states (seating gates, half-seated and off-axis fails, every torque
outcome) and replays both arms' demo plans through the kinematics at 20 Hz (every joint inside the URDF limits,
IK error under 0.5 mm). Expected output:

```
scoring: 29 cases pass (set torque 3.01 N*m = 14.0 N at the grip, reached 4.9 deg past snug; tolerance 90-110 % = 3.9-5.9 deg; yield 6.92 N*m = 32.2 N at 17.7 deg; real joint x1.0: 3 N*m set; seated = 2.8 of 3.5 mm engaged, tilt <= 8 deg, coax <= 1 mm)
left: 34.0 s plan, worst IK 0.099 mm / 0.014 deg
right: 34.0 s plan, worst IK 0.099 mm / 0.012 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/click-wrench-stop-at-click/click_wrench_stop_at_click.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick up the torque wrench standing on its rest, seat its socket fully on the lug bolt, and torque the bolt with it. The wrench is already set to the bolt's torque. Then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/click-wrench-stop-at-click-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/click-wrench-stop-at-click/click_wrench_stop_at_click.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick up the torque wrench standing on its rest, seat its socket fully on the lug bolt, and torque the bolt with it. The wrench is already set to the bolt's torque. Then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/click-wrench-stop-at-click-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 34 s of plan, both arms, built on two
rules a fitter follows with a heavy-headed wrench on a small vertical bolt: never move or leave it one-handed,
and press the head down while pulling. Both hands lift the tool off its rest - the left pinching the steel
neck, the right gripping the rubber grip - carry it level over the joint holding its clocking, and lower it
together until the chamfers meet, the hex is home (no overdrive) and the grip is down on the handle rest.
The right keeps hold while the left swaps to pressing the head straight down on the drive axis (flush ring
selector: nothing to snag; the press's sliding friction works at a few millimetres' radius, a few hundredths
of a N*m on the click); only then does the right let go, straddle the grip - side pushes only, so no finger
moment bypasses the breakover and the detent carries the full drive torque, while the rest's friction acts at
the pull radius and cancels out of the detent's load - and pull it round until the wrench clicks (spec angle +
breakover deflection + straddle slack + the arm's lag under load), stopping short of driving past the click and
easing back 1.5 deg. Both hands let go and park, over 10 cm outside the wrench's bounding box, so the success
hold (1 s) only starts once the release is plain in the video; the torqued wrench stays seated with its grip
on the rest. (`VARIANT = "pretest"` is a carry-robustness self-check: two grasp-carry-return cycles, the
second with both grasp points deliberately offset; it never touches the bolt.)

## Control runs (ablations)

Set `VARIANT` at the top of the task file:

- `"half"`: the socket is set down 2.5 mm off the bolt's axis, cocked on the head and never seated (coax
  over the 1 mm gate), and the right pulls anyway: no seating credit, any click uncredited, score 0.
  Once physical overlap begins, its wrench-head marker turns before the full seat latch: PRESEAT-TURN fires and
  the trial scores 0. The offline negative control holds the bolt fixed while turning only the half-seated wrench.
- `"through"`: the naive pull straight on through the click to 45 deg: OVER-TORQUED latches, score 0.
- `"short"`: stops before the click: under-torqued, at most 0.25.
- `"sweep"`: a sim-calibration probe, not an ablation: a slow pull while the breakover drive steps
  through four commanded stiffnesses, used to bracket-fit `BREAKOVER_DRIVE`; it never scores.

## Videos and screenshot

```bash
for log in ~/yam-eval/click-wrench-stop-at-click-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/click-wrench-stop-at-click-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Back the bolt off (fit a new bolt if its peak went over 110 % or the head was rounded); run it down by hand, then
to 1.5 N*m on the digital adapter, zero the camera markers, clear the adapter's peak memory, check the wrench at
3 N*m against the same adapter, and stand it back on its bench rest, socket down, grip on the block, handle toward
the right arm.
