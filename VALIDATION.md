
# Flight Dynamics Simulator — Validation

## 1. Objective

Evaluate basic numerical stability, flight control
responses, autopilot tracking, and graphical functionality
of the C++ Flight Dynamics Simulator.

This is software-level validation against project-defined
test conditions, not certification of a real aircraft model.

## 2. Build Verification

Platform: Windows

Compiler: GCC / MinGW

C++ Standard: C++17

Build System: CMake

Build command:

```powershell
cmake -S . -B build -G "MinGW Makefiles"
cmake --build build -j 4
```

Build result: TO BE VERIFIED

## 3. Automated Testing

Test command:

```powershell
ctest --test-dir build --output-on-failure
```

Expected test suites:

1. FlightPhysicsTests
2. FlightControlTests
3. FlightAutopilotTests
4. FlightTelemetryTests
5. FlightConfigurationTests
6. FlightSafetyTests
7. Flight6DOFTests
8. Flight6DOFControlCameraTests
9. Flight6DOFAutopilotTests

Actual results:

- Tests passed: TO BE RECORDED
- Tests failed: TO BE RECORDED
- Test date: TO BE RECORDED

Record the actual CTest output before marking the
release as validated.

## 4. Autopilot Flight Scenarios

Run:

```powershell
.\build\flight_6dof_scenarios.exe
```

The scenario runner evaluates:

| Scenario | Altitude Change | Heading Change |
|----------|-----------------|----------------|
| Level hold | 0 ft | 0 deg |
| Climb | +100 ft | 0 deg |
| Right turn | 0 ft | +20 deg |
| Combined maneuver | +100 ft | +20 deg |

Recorded measurements include:

- Final altitude error
- Final heading error
- Maximum bank angle
- Minimum airspeed
- Flight trajectory over time
- Control-surface commands

### Scenario Results

Enter values from the current Version 13 scenario run.

| Scenario | Altitude Error (ft) | Heading Error (deg) |
|----------|---------------------|---------------------|
| Level hold | Pending | Pending |
| Climb | Pending | Pending |
| Right turn | Pending | Pending |
| Combined maneuver | Pending | Pending |

### Evaluation

Assess whether each scenario:

- Reaches its intended altitude
- Reaches its intended heading
- Levels off without persistent large oscillations
- Maintains reasonable airspeed
- Produces finite, continuous state values

Final tracking accuracy alone is not sufficient
to determine settling time or overshoot.

## 5. Graphical Functionality

Manual checks for the Version 13 application:

- [ ] 3D aircraft appears
- [ ] Aircraft responds to elevator
- [ ] Aircraft responds to ailerons
- [ ] Aircraft responds to rudder
- [ ] Throttle input works
- [ ] Artificial horizon responds to pitch
- [ ] Artificial horizon responds to bank
- [ ] Airspeed and altitude readings update
- [ ] Sky, clouds, and terrain appear
- [ ] Autopilot can be engaged and disengaged
- [ ] Manual input overrides autopilot
- [ ] F2 settings menu works
- [ ] Restart and pause work
- [ ] CSV telemetry is generated

## 6. Limitations

The numerical aircraft model and autopilot have been
developed for educational simulation.

The current project does not establish:

- Real-world aerodynamic accuracy
- Aircraft certification compliance
- Real-world autopilot safety
- Accurate stall recovery in every condition
- Physical terrain collision in the 3D simulator

These limitations should be considered when
interpreting simulation results.

## 7. Release Decision

Release status: PENDING VALIDATION

Only change this to VALIDATED FOR PROJECT RELEASE
after all relevant tests and manual checks pass.
```
