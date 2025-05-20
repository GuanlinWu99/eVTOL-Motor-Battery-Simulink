# eVTOL Simulator for FDCHSL ABMS Project

Repository for eVTOL simulation leveraging MATLAB-Simulink

## Get Started

You may refer to the MATLAB provided tutorial:

```MATLAB
open utilities\TuneControlDesignForUAVInHoverExample.mlx
```

From the project root directory, open the MATLAB-Simulink Project:

```MATLAB
prj = openProject("VTOLRefApp.prj");
```

Opening the project already runs <code>setupPlant.m</code>. 

## Run Working Model

From the project root directory, run the following command:

```MATLAB
run src\main.m
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