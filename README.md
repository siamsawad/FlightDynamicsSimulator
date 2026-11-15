
# Flight Dynamics Simulator

A modular flight dynamics and control simulator developed
in C++17, featuring six-degree-of-freedom aircraft physics,
interactive 3D visualization, and automatic flight control.

The project demonstrates numerical simulation, rigid-body
dynamics, control systems, graphics programming, and
software testing.

## Features

### 1. Six-Degree-of-Freedom Flight Dynamics

The simulator models aircraft motion using:

- Three-dimensional position and velocity
- Roll, pitch, and yaw dynamics
- Quaternion-based aircraft orientation
- Aerodynamic forces and moments
- Lift, drag, thrust, and gravity
- Atmospheric density and wind inputs
- Fixed-timestep numerical integration

The simulator uses a North-East-Down (NED) world
coordinate system and a Forward-Right-Down (FRD)
aircraft body coordinate system.

### 2. Interactive 3D Visualization

The graphical simulator includes:

- A three-dimensional aircraft model
- Perspective projection
- A chase camera
- Real-time aircraft rotation and movement
- A flight-path trail
- Stylized terrain, sky, and clouds

Graphics are implemented using SFML 3.1.

### 3. Primary Flight Display

The PFD displays:

- Artificial horizon
- Pitch and bank angle
- True airspeed
- Altitude
- Vertical speed
- Heading

The artificial horizon responds to the aircraft's
actual simulated orientation.

### 4. Flight Control System

Keyboard controls provide:

- Elevator
- Ailerons
- Rudder
- Throttle

Control-surface commands pass through rate-limited
actuators before reaching the flight dynamics model.

### 5. Autopilot

The 6-DOF autopilot supports:

- Altitude hold
- Heading hold
- Climb and descent commands
- Bank-angle control
- Pitch-rate and roll-rate damping
- Simplified low-airspeed and high-angle-of-attack
  responses
- Manual flight-control override

Engine throttle remains manually controlled.

### 6. Telemetry and Automated Testing

The project includes:

- Flight telemetry exported to CSV
- Automated flight scenarios
- Physics unit tests
- Control-system tests
- Autopilot integration tests
- Configuration and safety-model tests

## Technologies

| Technology | Purpose |
|------------|---------|
| C++17 | Core implementation |
| CMake | Build system |
| SFML 3.1 | Graphics and user input |
| Git | Version control |
| CSV | Flight data export |

## Build Instructions

### Requirements

- Windows
- C++17-compatible MinGW/GCC compiler
- CMake 3.28 or newer
- Git (used by CMake to fetch SFML)

### Configure

```powershell
cmake -S . -B build -G "MinGW Makefiles"
```

### Build

```powershell
cmake --build build -j 4
```

### Run the 3D Simulator

```powershell
.\build\flight_6dof_gui.exe
```

### Run Automated Tests

```powershell
ctest --test-dir build --output-on-failure
```

### Run Flight Scenarios

```powershell
.\build\flight_6dof_scenarios.exe
```

## Keyboard Controls

| Key | Action |
|-----|--------|
| Up / Down | Pitch control |
| A / D | Roll control |
| Left / Right | Roll control |
| Q / E | Rudder control |
| W / S | Throttle |
| H | Toggle autopilot |
| Page Up / Page Down | Adjust altitude target |
| J / L | Adjust heading target |
| F2 | Aircraft settings |
| P | Pause |
| R | Restart |
| Esc | Exit |

## Software Architecture

The application separates flight physics, controls,
autopilot logic, graphics, and testing.

```text
Pilot / Autopilot Commands
          |
          v
  Rate-Limited Actuators
          |
          v
   Aerodynamic Forces
          |
          v
      6-DOF Model
          |
          v
   Aircraft State
          |
          +----> 3D Renderer and PFD
          |
          +----> CSV Telemetry
```

## Project Structure

```text
FlightDynamicsSimulator/
|
|-- include/FlightSim/
|   |-- Dynamics6DOF.hpp
|   |-- Controls6DOF.hpp
|   |-- Autopilot6DOF.hpp
|   |-- Projection3D.hpp
|   |-- Renderer6DOF.hpp
|   |-- Visuals13.hpp
|
|-- src/
|   |-- Dynamics6DOF.cpp
|   |-- Controls6DOF.cpp
|   |-- Autopilot6DOF.cpp
|   |-- Projection3D.cpp
|   |-- Renderer6DOF.cpp
|   |-- Visuals13.cpp
|   |-- flight_6dof_gui.cpp
|   |-- flight_6dof_scenarios.cpp
|
|-- tests/
|-- docs/
|-- CMakeLists.txt
|-- README.md
```

The project also retains the original two-dimensional
flight simulator and its test modules.

## Validation

Automated tests and flight scenarios evaluate the
behavior of the physics and control systems.

Testing procedures and measured results are documented
in `docs/VALIDATION.md`.

## Known Limitations

This is an educational flight dynamics simulator, not
a certified flight training or engineering tool.

- Aerodynamic coefficients are simplified.
- Aircraft behavior has not been validated against
  a specific real-world aircraft.
- The 3D simulator does not implement runway collision
  or landing gear dynamics.
- Environmental scenery is decorative, not geographic
  terrain.
- The autopilot does not control throttle automatically.
- The PFD is a simplified instrument display.

## Project Goals

This project was developed to practice and demonstrate:

- Object-oriented C++ design
- Numerical modeling and simulation
- Rigid-body dynamics
- Flight control algorithms
- Software architecture
- Automated testing
- Real-time visualization
```
