# Scavenge Bot

A small 4-servo desk robot arm, 3D printed on a Prusa MK3 from spare parts, built in public to test how well an AI
(Claude) can design a real machine and then control it through a camera.

## Status

- All parts printed and assembled; calibrated in software (no re-seating servo horns).
- First grab-and-lift by Claude on 17 Nov 2026, driving the arm over serial and watching through a phone camera held overhead.
- Next: a fixed overhead camera and a small tool server so Claude can send commands and see two cameras directly.

## How it's built

- **No glue, no pins.** Every joint is screwed together with printed screws in a house thread designed for the printer:
  8 mm, 2 mm pitch, 45° flanks. The 45° angle prints cleanly without supports.
- **Servo horns** sit in a printed slot and are clamped by a screw-on backplate, so the horn can't twist or pull out.
- **Servo pockets** have 0.4 mm clearance and wire notches at both ends.
- **Gripper:** two geared jaws (module 1.5, 16 teeth) driven by one SG90, mounted sideways so the jaws open across the arm.
  Rubber bands on the jaw tips made the difference for grip.

## Print list (`parts/`)

All PLA, 0.30 mm DRAFT, 15% infill, no supports.

| File | Qty | Notes |
|---|---|---|
| arm_base.stl | 1 | Exported upside down (top plate on the bed) |
| arm_base_lid.stl | 1 | Bosses up |
| house_screw_19mm_base.stl | 4 | Head down, brim |
| shoulder_turntable.stl | 1 | Disc down |
| turntable_backplate.stl | 1 | Plate down |
| upper_arm.stl | 1 | Flat |
| upper_arm_backplate.stl | 2 | One for the upper arm, one for the forearm |
| forearm.stl | 1 | Flat |
| gripper_base_v2.stl | 1 | Plate down, foot up (teardrop holes) |
| gripper_jaw_driven.stl | 1 | Slot side up |
| gripper_jaw_idler.stl | 1 | Flat |
| gripper_top.stl | 1 | Flat |
| house_screw_9mm.stl | 9 | Turntable 2, upper arm 1, forearm 2, gripper 4. Head down |
| house_screw_9mm_slimhead.stl | 0–1 | Optional, upper arm outer hole (a full head hits the elbow servo tab) |

`parts/tests/` has the fit tests (threads, horn slots, gear mesh).

## Assembly notes

- **Servo orientation:** tabs go on the face the shaft comes out of. The shoulder shaft goes at the top of its pocket,
  and the elbow shaft goes at the far end of the upper arm. The base servo goes in from the top, shaft up.
- **Horns:** horn into its slot, then backplate on, then screws, then press onto the shaft, then the horn screw through the access hole.
- **Gripper:** servo into the base, base onto the forearm (2 screws), horn on, both jaws, top plate (2 screws) last.

## Electronics and firmware

Arduino Uno on USB; servos on pins 3 (base), 5 (shoulder), 6 (elbow), 9 (gripper), powered from 4×AA through breadboard
rails with a 1000 µF capacitor across them; Uno ground tied to servo ground; nothing on the Uno's 5V pin.

`electronics/arduino/scavenge_bot/` is the firmware (115200 baud, one command per line):
- `home`, `move B S E G`, `base|shoulder|elbow|gripper N`, `open`, `close`, `relax`, `speed N`, `status`
- calibration stored in EEPROM: `cal start`, `nudge J N`, `cal J min|max|ref`, `cal J dir 1|-1`, `cal save`
- safe waking: servos boot relaxed; `creep J T` twitches a joint toward T so you can see where it really is, `hold J T` wakes it there.
  (A full-speed wake once broke a servo's end stop.)

## Bill of materials (approx. US prices)

| Item | Cost |
|---|---|
| PLA, ~250 g including tests | $4–6 |
| 4× SG90 micro servos | $8–15 |
| Arduino Uno (clone / genuine) | $12–15 / ~$27 |
| Half breadboard + jumpers | $8–12 |
| 4×AA holder with switch + batteries | $5–9 |
| 1000 µF 25 V capacitor | $1 |
| **Total** | **~$40–60** |

## Digital twin (`twin/`)

A browser 3D model of the arm built from the real STLs: joint sliders, click-to-reach inverse kinematics, collision
warnings, the electronics with routed cables, a virtual Arduino serial monitor and a power and load estimate.
It caught a real design flaw before printing: the first gripper's jaws opened up and down, into the desk.
Serve the folder locally (`python3 -m http.server`) and open `index.html`.

## Tools

- `cad/`: Python generators (manifold3d) for the forearm and gripper. The earliest parts are STL-only.
- `tools/slicer/`: PrusaSlicer command-line check with the stock MK3 draft profile, plus a G-code report of overhangs and bridges.
- `tools/tests/`: automated twin checks.

## Lessons so far

Worked first try: the house thread, the horn slot and backplate, the gear mesh.
Needed fixes: the servo wire bulge (notches), a screw head hitting a servo tab, a guessed servo height (measure first),
the gripper jaw plane (the twin caught it), a mesh sliver from coincident faces, round horizontal holes (now teardrops).
Controlling it: a side-on camera hides depth (aim was off sideways until the view came from above); a blind servo wake
drives to the stored number wherever the joint really is; grip needed rubber bands.
Dead ends: cross horns, snap fits, filament pins.

## Licence

MIT, see `LICENSE`.
