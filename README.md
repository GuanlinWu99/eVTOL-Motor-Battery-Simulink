# eVTOL Simulator for FDCHSL ABMS Project

Repository for eVTOL simulation leveraging MATLAB-Simulink

## Required MATLAB packages

* UAV Toolbox
* Control System Toolbox
* Aerospace Blockset
* Aerospace Toolbox
* Simulink
* Simulink Control Design

## Get Started

You may refer to the MATLAB provided tutorial:

```MATLAB
open utilities\TuneControlDesignForUAVInHoverExample.mlx;
```

From the project root directory, open the MATLAB-Simulink Project:

```MATLAB
prj = openProject("VTOLRefApp.prj");
```

Opening the project already runs <code>setupPlant.m</code>. 

## Run Working Model

From the project root directory, run the following command:

```MATLAB
run src\main.m;
```

This may take some time as the simulation runs in the background without visualization.

## Automated Tuning for Hover Configuration

At the moment, <code>src/main.m</code> must be run prior to this to load necessary parameters. Run the following:

```MATLAB
run setupHoverManual;
set_param([mdl '/Manual Control Dashboard/Slider1'],'Value','10');
set_param([mdl '/Manual Control Dashboard/Slider2'],'Value','0');
set_param([mdl '/Manual Control Dashboard/Slider3'],'Value','0');
run exampleHelperAutomatedHoverControlTuning;
```

This may take some time as the automated tuning functions run.

## Fixes and Updates for Controller/Simulator

| Fix/Bug No. |               Name                | Location                                                                                                                                                                                                                                                                                                           | Description                                                                                                                                                                                                                                                                                    | Fixed |
| :---------: | :-------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----- |
|      1      |  Simulation solver set up error   | - VTOLDynamicsData.sldd                                                                                                                                                                                                                                                                                            | - Change VTOLTiltrotor.slx configuration to Fixed-step, ode4 (Runge-Kutta), step-size auto<br>- Change VTOLDynamics.slx configuration to  Fixed-step, ode4 (Runge-Kutta), step-size 0.001                                                                                                      | Y     |
|      2      |       PWM conversion error        | - VTOLTiltrotor/Autopilot/Controller (VTOLAutopilotController)/Low level controller/Scheduler/Subsystem/Actuator To Voltage                                                                                                                                                                                        | - Normalize the rotor speed commands to PWM<br>- Match I/O with dynamics module                                                                                                                                                                                                                | Y     |
|      3      | Actuator command conversion error | - VTOLTiltrotor/Autopilot/Controller (VTOLAutopilotController)/Low level controller/Scheduler/Subsystem/Switch Case Action Subsystem                                                                                                                                                                               | - Correct the computation of rotor speed commands from required thrusts<br>- Modify the blocks not to use w_trim, which is configuration-dependent parameter                                                                                                                                   | Y     |
|      4      |     Landing gear instability      | - VTOLTiltrotor/Digital Twin/UAV Dynamics (VTOLDynamics)/Force and Moments/Ground Model                                                                                                                                                                                                                            | - Imperfect landing gear model leading to the divergence when the VTOL is on ground<br>- Needs geometric configuration of landing gear model and update on moment model                                                                                                                        | N     |
|      5      | Aerodynamic /Inertia model update | - VTOLTiltrotor/Digital Twin/UAV Dynamics (VTOLDynamics)/Force and Moments/Aerodynamic Forces/Compute Body Frame  Forces and Moments                                                                                                                                                                               | - Center of pressure (CP), Center of gravity (CG) information need to be updated<br>- To be updated using OpenVSP analysis results<br>- Aerodynamic moments created at low velocity can create issues (clipping needed)                                                                        | N     |
|      6      |   VTOL attitude control scheme    | - VTOLTiltrotor/Autopilot/Controller (VTOLAutopilotController)/Low level controller/Multicopter Controller<br>- VTOLTiltrotor/Autopilot/Controller (VTOLAutopilotController)/Low level controller/Scheduler/Subsystem/Switch Case Action Subsystem                                                                 | - Disable tilt command in VTOL mode yaw attitude controller<br>- Enable the yaw control by the motors in the allocation module<br>- Enable the pitch control by the motors in the allocation module<br>- Use P controller for attitude feedback loop and PID controller for rate feedback loop | Y     |
|      7      |   VTOL position control scheme    | - VTOLTiltrotor/Autopilot/Controller (VTOLAutopilotController)/Low level controller/Multicopter Controller/Horizontal Position Control/XY Controller<br>- VTOLTiltrotor/Autopilot/Controller (VTOLAutopilotController)/Low level controller/Multicopter Controller/Attitude & Altitude controller/Altitude Control | - Disable forward position control using tilt command<br>- Use P loop for position feedback loop and PID controller for velocity feedback loop                                                                                                                                                 | Y     |