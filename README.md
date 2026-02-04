# rhex_motor_testing

## SETUP
When SSH and without wifi testing run commands:
ssh -t rhex@192.168.1.182 "sudo timedatectl set-ntp false; sudo date -s \"$(date '+%Y-%m-%d %H:%M:%S')\""

to get the correct date and time

# To SSH
rhex@192.168.2.110

## SFTP sync + build/run
SFTP sync (from your IDE)
- Verify the IP in `.vscode/sftp.json`.
- Right click in VS Code: Sync Local -> Remote.
- Ctrl+S also syncs to the Pi when SFTP is configured.
- The `sftp.json` file must match on both machines or sync will not work.

Build on the Pi
```
cd /home/rhex_motor_testing/build
cmake ..
make clean
make
```

Run the executable from `build/` based on the CMake target (each target includes
`src/cubemars_pi3hat.cpp` + `src/pi3hat.cpp` plus the specific stand_sit file):
- `rhex_tripod_test`: `src/stand_sit_tripod_main_ames.cpp`
- `rhex_tripod_vel`: `src/stand_sit_tripod_vel_main_test.cpp`
- `rhex_tripod_vel_ames`: `src/stand_sit_tripod_vel_main_test_ames.cpp`
- `rhex_stand_sit_test`: `src/stand_sit_main.cpp`
- `rhex_stand_sit_belly_test`: `src/stand_sit_belly_main.cpp`
- `rhex_stand_sit_belly_tripod_test`: `src/stand_sit_belly_main_tripod.cpp`

Example:
```
sudo ./rhex_stand_sit_test --zero
```

Note: If a target fails to configure or build, confirm the source file named in
`CMakeLists.txt` exists or update the target mapping.

## Code map (stand/sit + gait programs)
Backbones (core patterns reused)
- `src/stand_sit_main.cpp`: base stand/sit only. Core state machine (hold stand, return home, hold home, exit), smooth ramp to stand, home capture, and MIT mode setup. This is the backbone for the rest.
- `src/stand_sit_tripod_main.cpp`: position-control tripod gait layered on the same stand/sit flow. Adds interactive commands and `AdvanceTripodPhase` with phase stepping and debug handoff snapshots.
- `src/stand_sit_tripod_vel_main.cpp`: velocity-control tripod gait. Uses SmoothStep + derivative to compute phase velocity, clamps velocity, wraps phases for multi-step gait.
- `src/stand_sit_belly_main.cpp`: stand, then belly-flop gait with velocity commands and non-blocking keyboard input. This is the backbone for belly tests.

Variants / tests
- `src/stand_sit_tripod_main_ames.cpp`: same structure as `stand_sit_tripod_main.cpp` with AMES gains and tuned constants.
- `src/stand_sit_tripod_vel_main_test_ames.cpp`: velocity + unwrap logic to allow multi-turn rotation. Tracks continuous position with `UnWrapState` and `UpdateUnwrapRange`.
- `src/stand_sit_tripod_vel_main_test_ames_v2.cpp`: v2 of the velocity test. Adds a `TwoStrokeClock` and a stub `RunTwoStrokeTripod` (not fully implemented yet), plus the same velocity phase control.
- `src/stand_sit_tripod_torque_main_test_ames.cpp`: torque-based tripod gait with unwrap logic and torque limits (`kTauMax`).
- `src/stand_sit_belly_main_ames.cpp`: AMES-tuned belly gait (different gains and shorter ramp time).
- `src/stand_sit_belly_tripod_main_test.cpp`: belly-only tripod velocity test with two gait modes (offset-sine vs alternating) and keyboard controls.
- `main.cpp`: empty placeholder.

## Common motor + CAN setup (in most stand_sit* files)
- CAN config: each main builds `mjbots::pi3hat::Pi3Hat::Configuration`, sets `config.can[4].slow_bitrate = 1000000`, `fdcan_frame = false`, `bitrate_switch = false`, then disables `config.can[0..3].slow_bitrate = 0`. This targets the Pi3Hat low-speed bus (JC5).
- CLI args: `--bus N` selects the `can_bus` index (default 4) passed into each `CubemarsPi3Hat` instance; `--zero` zeroes encoders before home capture.
- Motor IDs: six CubeMars motors are constructed with IDs 10-15.
- Tripod grouping: left tripod = {13 (L), 10 (L), 11 (R)}; right tripod = {12 (R), 15 (R), 14 (L)}.
- Typical sequence: `enterMITMode()` for all motors -> `PrimeFeedback()` to refresh reads -> optional `zeroMotor()` per motor -> `CaptureHome()` to store current encoder positions -> ramp to stand (`MoveTripodsToOffset()`) -> state machine loop.
- Position limits: most programs clamp commanded position to +/-12.5 rad (`kPMin/kPMax`).
- Direction convention: each file defines its own `SideSign` (left vs right). Some files use left=+1, others left=-1, depending on how direction is defined for that test.

## Controls and modes
- Tripod position/velocity/torque programs (most `stand_sit_tripod_*` files) start a command thread and accept: `stand | tripod | home | holdhome | exit`.
- Belly programs use non-blocking keyboard input:
  - `stand_sit_belly_main.cpp` and `stand_sit_belly_main_ames.cpp`: code starts gait on key `t` (message still says g/space), then `w/s` adjust speed and `a/d` adjust cadence.
  - `stand_sit_belly_tripod_main_test.cpp`: `g`/space starts; `w/s` adjust speed; `a/d` adjust cadence; `1` selects offset-sine; `2` selects alternating.

## Low-level CAN/MIT implementation (include/ + src)
- `include/pi3hat.h` / `src/pi3hat.cpp`: Pi3Hat API and CAN bus definitions. The header notes buses 1-4 map to JC1-JC4 (high speed), and bus 5 maps to JC5 (low speed).
- `include/cubemars_pi3hat.h` + `src/cubemars_pi3hat.cpp`: CubeMars MIT mode wrapper.
- Parameter limits used for packing: position +/-12.5 rad, velocity +/-50 rad/s, torque +/-65, kp 0-500, kd 0-5.
  - `sendCommandMITMode()` packs an 8-byte CAN frame per motor and expects a reply; it unpacks position/velocity/torque/temperature/error into cached `motor_data_`.
- `enterMITMode()` / `exitMITMode()` / `zeroMotor()` send the MIT special frames (0xFF..FC/FD/FE).
  - Frames are sent on Pi3Hat bus 5 (JC5) with `force_can_check = (1 << 5)`; the `can_bus` argument selects which slot in the tx/rx arrays to use.
