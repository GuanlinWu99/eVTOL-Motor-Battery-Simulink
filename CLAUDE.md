# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an eVTOL (electric Vertical Take-Off and Landing) simulator for the FDCHSL ABMS Project built using MATLAB-Simulink. The simulator models a tilt-rotor VTOL aircraft with battery-electric powertrain, supporting multiple flight modes: Hover, Forward Transition, Fixed Wing (Cruise), and Back Transition.

## Required MATLAB Packages

- UAV Toolbox
- Control System Toolbox
- Aerospace Blockset
- Aerospace Toolbox
- Simulink
- Simulink Control Design

## Development Commands

### Starting the Simulator

1. **Open MATLAB Project** (initializes environment):
   ```matlab
   prj = openProject("VTOLRefApp.prj");
   ```
   Note: Opening the project automatically runs setup scripts.

2. **Run Full-Scale Simulation**:
   ```matlab
   run src/FullScale/startup.m
   ```
   This is the main entry point for running simulations. It:
   - Loads the VTOLTiltrotor model
   - Initializes flight parameters and configurations
   - Runs the simulation and generates plots for battery performance, flight dynamics, and rotor performance

3. **Access Tutorial Example**:
   ```matlab
   open utilities/TuneControlDesignForUAVInHoverExample.mlx
   ```

### Control Tuning

For automated hover control tuning, first run the main simulation to load parameters, then:
```matlab
run setupHoverManual;
set_param([mdl '/Manual Control Dashboard/Slider1'],'Value','10');
set_param([mdl '/Manual Control Dashboard/Slider2'],'Value','0');
set_param([mdl '/Manual Control Dashboard/Slider3'],'Value','0');
run exampleHelperAutomatedHoverControlTuning;
```

## Architecture Overview

### Directory Structure

- **`models/`**: Core Simulink models
  - `VTOLTiltrotor.slx` - Top-level tilt-rotor aircraft model
  - `VTOLAutopilotController.slx` - Autopilot controller subsystem
  - `VTOLDynamics.slx` - UAV dynamics/physics subsystem
  - `Motor_batt/` - Battery and electric motor models including PID control
  - `mysimcape_lib/` - Custom Simscape libraries for battery equivalent circuit models

- **`src/FullScale/`**: Main simulation source for full-scale aircraft
  - `startup.m` - Primary simulation script (run this to execute simulation)
  - `parameters/` - Aircraft parameters and configuration loaders
    - `load_vtol_dynamics_7000lb.m` - Loads 7000 lb aircraft configuration
    - `load_controller_parameters.m` - Controller parameter setup
    - `load_controller_gains.m` - Control system gains
  - `mission/` - Mission profile definitions (e.g., P4mission.m)
  - `ctrldesign/` - Control system design files
  - `aero/`, `trim/` - Aerodynamics data and trim calculations

- **`src/full_scale/`**: Setup scripts for different configurations
  - `setupHoverConfiguration_mod.m` - Hover mode initialization
  - `setupTransitionGuidanceMission_mod.m` - Transition mission setup
  - `setupFixedWingConfiguration_mod.m` - Fixed-wing mode setup

- **`utilities/`**: Helper functions organized by purpose
  - `Setup/` - Initialization and configuration helpers
  - `Mission/` - Mission planning utilities
  - `ControlTuning/` - Automated tuning functions
  - `Visualization/` - Plotting and animation tools

- **`data/`**: Simulation data files including:
  - `VTOLDynamicsData.sldd` - Simulink data dictionary
  - `contact.mat` - Landing gear contact model
  - Tuned control gains (`.mat` files)

- **`work/`**: Generated Simulink cache and build artifacts

### Flight Modes

The aircraft operates in four distinct modes defined as enumerated type `flightState`:
- **Hover (0)**: Vertical flight with rotors oriented upward
- **Transition (1)**: Forward transition from hover to cruise
- **FixedWing (2)**: Cruise flight with rotors tilted forward
- **BackTransition (3)**: Transition from cruise back to hover

### Key System Components

1. **Aircraft Configuration**:
   - Tilt-rotor design with 4 motors (rotors 1 & 2 tilt, 3 & 4 fixed)
   - 7000 lb full-scale configuration with scaled aerodynamic coefficients
   - Control surfaces: ailerons, elevators, rudder, flaps

2. **Control System Architecture**:
   - Hierarchical control: Guidance → Autopilot → Low-Level Controller → Scheduler
   - Mode-specific controllers: Multicopter controller for hover/transition, fixed-wing controller for cruise
   - Attitude control: P-loop for attitude, PID for rate control
   - Position control: P-loop for position, PID for velocity control

3. **Powertrain Model**:
   - Battery pack with equivalent circuit model (Rcecm)
   - Four electric motors with drag torque modeling
   - PID motor speed controllers
   - Outputs: SOC, C-rate, current, voltage, power, energy consumption

4. **Dynamics Simulation**:
   - Fixed-step solver (ode4, Ts=0.001s)
   - 6-DOF aircraft dynamics with aerodynamic forces/moments
   - Landing gear ground contact model
   - Wind disturbance capability (toggleable)

### Mission Profile System

Missions are defined as structs with waypoints specifying:
- `mode`: Flight mode at waypoint
- `position`: [North, East, Down] in meters
- `params`: Mode-specific parameters (e.g., airspeed for fixed-wing)

Example: `src/FullScale/mission/P4mission.m` defines a complete mission including takeoff, climb, cruise, descent, and landing.

## Simulation Configuration

- **Solver**: Fixed-step, ode4 (Runge-Kutta), step size 0.001s
- **Initial Conditions**: Set in startup.m (position, attitude, velocity)
- **Flags**:
  - `Visualization` (0/1): Enable/disable 3D visualization
  - `Wind` (0/1): Enable/disable wind disturbance
  - `SensorType` (0/1): Ideal vs realistic sensors
  - `TuningMode` (0/1): Normal operation vs tuning mode

## Common Development Patterns

### Adding a New Mission Profile

1. Create mission file in `src/FullScale/mission/` following the TransitionMission struct format
2. Call the mission setup in `startup.m` after parameter initialization
3. Ensure waypoints specify appropriate flight modes and positions

### Modifying Aircraft Parameters

Aircraft mass, geometry, aerodynamics, and inertia are loaded via `load_vtol_dynamics_7000lb.m`. Modify this function to change physical parameters. Note the scaling factors `n` (length) and `sigma` (density) for geometric scaling.

### Tuning Controllers

Control gains are loaded in `load_controller_gains.m` from `.mat` files in `data/`. Use automated tuning utilities in `utilities/ControlTuning/` or manually adjust gains. Always test in hover mode first before attempting transition/cruise.

### Working with Battery Models

Battery parameters are in `HEV_Param` struct loaded by `load_vtol_dynamics_7000lb.m`. The equivalent circuit model (Rcecm) is in `models/mysimcape_lib/`. Modify cell parameters, pack configuration (series/parallel), or thermal models here.

## Known Issues

Refer to the table in README.md (Fixes and Updates section) for documented bugs and fixes. Key resolved issues:
- Simulation solver configuration (Fixed)
- PWM conversion errors (Fixed)
- VTOL attitude and position control schemes (Fixed)

Outstanding issues:
- Landing gear instability during ground contact
- Aerodynamic/inertia model needs updates from OpenVSP analysis
- Low-velocity aerodynamic moments require clipping

## Git Workflow

Current branch: `fullscale`
Main branch: `main`

Modified files are primarily in `src/FullScale/` and `models/`, indicating ongoing development of full-scale simulation features and transition control.
