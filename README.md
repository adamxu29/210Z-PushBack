# 210Z-PushBack

Competition robot firmware for **VEX V5RC Team 210Z**, built for the *Push Back* season. This repo is the robot's brain: drivetrain control, a custom localization/motion stack ("Lib-Eclipse"), a velocity-controlled scoring mechanism with anti-jam and color-sort logic, and an LVGL touchscreen GUI for the V5 Brain — all running on top of [PROS](https://pros.cs.purdue.edu/).

## Technical features

**Localization** — two complementary systems run side by side:
- **Dead-wheel odometry** (`Odom`) — tracks robot pose from vertical/horizontal tracking wheels + IMU heading, with periodic position resets against field walls using the four VEX distance sensors (`distance_sensor_reset`).
- **Monte Carlo Localization** (`MCL`) — a 300-particle filter with a structure-of-arrays layout for cache efficiency and a hand-rolled `XorShift32` + Box-Muller PRNG (no heap allocation, no `<random>`), so it's cheap enough to run continuously on the V5 Brain's ARM Cortex. Each of the four wall-facing distance sensors has its own mount offset, noise sigma, and max range; particle likelihood is computed via ray-vs-AABB casts against the field geometry (goals, loader, center barrier) using Eigen.

**Motion control** — layered PID controllers rather than a single monolithic drive class:
- `Translation_PID`, `Rotation_PID`, `Curve_PID` — independent single-axis controllers for straight moves, point turns, and constant-radius curves, each supporting slew-rate limiting and *motion chaining* (settle-and-continue without a full stop between chained movements).
- `Drive` — a combined translation+rotation controller exposing `move_to_point()` / `turn_to_point()` for point-to-point autonomous movement.
- `PID` — a generic single-actuator controller (motor or motor+rotation-sensor feedback), reused as the correction term inside the intake's velocity controllers.
- `PID_Tuner` — an on-robot live tuner: cycle through kp/ki/kd and translation/rotation/curve from the controller, nudge constants, and re-run the move — no recompiling to tune a PID loop trackside.

**Scoring / intake** — `Intake` runs a velocity-controlled dual-roller system (intake + indexer):
- Feedforward `LookupTable` (RPM → voltage) plus a PID correction term drive each roller, tuned per direction from an empirically measured curve.
- A 3-state anti-jam state machine (`Idle` / `Outtaking` / `Intaking`) detects stalls via motor current draw and automatically reverses briefly to clear jams.
- Optical-sensor color sorting: `score_until_opp_block()` runs a roller until it detects an opponent-colored ball via the optical sensors, then stops or reroutes it automatically.
- Scoring modes (`Intake`, `MatchLoad`, `Outtake`, `Top`, `Middle`, `Bottom`, `None`) map directly onto trapdoor/mid-goal/intake-lift pneumatics.

**GUI** — an LVGL interface on the V5 Brain screen with five screens: Home, Autonomous Selector (alliance color + path list), Sensors (live position/encoder/motor-temp readout), Match Checklist (timer, battery, radio status, skills toggle), and a Debug Terminal (9-line scrolling log) for field-side debugging without a laptop tether.

**Autonomous routines** — a library of selectable paths (`solo_awp`, `anti_solo_awp`, `left/right_half_awp`, `left/right_63`, `left/right_7`, `left/right_9`, `skills`, `test`), chosen either through the GUI's autonomous selector or the driver-control skills toggle.

## Project layout

```
include/ src/
├── Lib-Eclipse/
│   ├── Chassis/
│   │   ├── Chassis_PID      Translation / Rotation / Curve PID controllers
│   │   ├── Drive            Combined move_to_point / turn_to_point
│   │   ├── Odometry         Dead-wheel odometry + distance-sensor resets
│   │   └── MonteCarloLocalization   Particle-filter localization
│   ├── PID                  Generic single-actuator PID
│   ├── Intake                Scoring mechanism: velocity control, anti-jam, color sort
│   ├── Utility               Drive constants, heading/error helpers, color-sort helpers
│   └── GUI                   LVGL touchscreen interface
├── OPControl/
│   ├── OPControlSystems      Driver control (arcade drive, button mappings)
│   └── PIDTuner              On-robot live PID constant tuner
├── Autonomous/
│   └── Autonomous-Paths      All selectable autonomous routines
├── Configuration/
│   └── Globals               Motor/sensor/pneumatic port wiring, object instances
└── main.cpp                   initialize() / autonomous() / opcontrol() entrypoints
```

`include/pros`, `include/display`, `include/okapi`, and `include/Eigen` are vendored dependencies (PROS API, LVGL, okapilib headers, and Eigen respectively).

Port wiring (`src/Configuration/Globals.cpp`) and the controller button mapping (`OPControl::driver_control()`) are both just current configuration, not part of the design — expect them to change as the robot's hardware and driver preferences evolve.

## Building & uploading

Requires the [PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/installing-cli.html).

```bash
git clone https://github.com/adamxu29/210Z-PushBack.git
cd 210Z-PushBack

# Build
make

# Upload to a connected V5 Brain
pros upload
```

Open `210Z-PushBack.code-workspace` in VS Code (with the PROS extension) for build/upload buttons and correct include paths.

## Configuring & extending

1. **Electronics** — every motor, sensor, and pneumatic port is declared in `src/Configuration/Globals.cpp`. Rewire a port there and the change propagates everywhere else through the `extern` declarations in `Globals.hpp` — no other file needs to change.

2. **GUI** — all screen logic lives in `Lib-Eclipse/GUI.cpp`, built on LVGL. Five screens:
   - **Home** — landing screen, navigates to the other four.
   - **Autonomous Selector** — pick alliance color and autonomous routine before a match.
   - **Sensors** — live position, encoder, and motor-temperature readout.
   - **Match Checklist** — match timer, battery level, radio status, skills toggle.
   - **Debug Terminal** — a 9-line scrolling log for field-side debugging without a laptop.

   New screens follow the same pattern: declare the `lv_obj_t*` widgets in `GUI.hpp`, then add an `initialize_*` / `display_*` pair in `GUI.cpp`.

3. **Autonomous paths** — routines are built from manually-plotted points rather than generated. Field coordinates can be found using [path.jerryio.com](https://path.jerryio.com/) (see the comment at the top of `Autonomous-Paths.cpp`), then hand-tuned via trial and error. New routines go in `Autonomous-Paths.cpp` / `.hpp`, using `solo_awp()`, `left_9()`, etc. as templates.

## Selecting & running autonomous

1. Power on the Brain — the GUI boots to the Home screen.
2. Select the program slot this build was uploaded to.
3. Go to the **Autonomous Selector** screen, pick an alliance color and a path (or enable Skills mode from the Match screen).
4. The selected routine runs automatically when the field/competition switch enters autonomous, or can be triggered manually from driver control with **X** for bench testing.

## Acknowledgments

- The `Lib-Eclipse` framework name and architecture are derived from Zechariah Wang's open-source [Eclipse Robotics Library](https://github.com/ZechariahWang/Eclipse-Robot_framework), adapted and extended for 210Z's hardware, localization approach, and scoring mechanisms for the High Stakes and Push Back Seasons.
- [PROS](https://pros.cs.purdue.edu/) — Purdue Robotics Operating System.
- [LVGL](https://lvgl.io/) — embedded graphics library powering the Brain GUI.
- [Eigen](https://eigen.tuxfamily.org/) — linear algebra used in the particle filter's sensor model.
