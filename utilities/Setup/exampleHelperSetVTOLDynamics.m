function uavParam=exampleHelperSetVTOLDynamics()
% exampleHelperSetVTOLDynamics set VTOL geometry and aerodynamics.

% Copyright 2023 The MathWorks, Inc.

%define Aerodynamics
%% Define Aerodynamics
uavParam.aero.rho=1.225;
uavParam.aero.CL0=0.81857;
uavParam.aero.CLa=4.09127;
uavParam.aero.CLa_dot=0;
uavParam.aero.CLq=7.31097;
uavParam.aero.CLDe=0.50787;
uavParam.aero.CLDa=-0.85291;
%Drag
uavParam.aero.CDmin=0.06047;
uavParam.aero.K= 0.1328;
uavParam.aero.CL_CDmin= 0.4806;
uavParam.aero.A1=0;
uavParam.aero.CdDe=0.016043;
uavParam.aero.CdDa=0.092132;
uavParam.aero.CdDr= 0.0097403;
%Moment coefficient
uavParam.aero.Cm0=0.00763;
uavParam.aero.Cma=-1.76966;
uavParam.aero.Cmq=-19.22663;  
uavParam.aero.CmDe=-1.83747;
%%uavParam.aero.CmDr= 0.02991

%Side force
uavParam.aero.CYb=-0.001570;
uavParam.aero.CYp=-0.001570;
uavParam.aero.CYr=0.18368;
uavParam.aero.CYDr=0.128915;

%Roll moment
uavParam.aero.Clb=-0.001610;
uavParam.aero.Clp=-0.47584 ;
uavParam.aero.Clr= 0.27153;
uavParam.aero.ClDa=  0.286593;
uavParam.aero.ClDr=  0.013407;

%Yaw moment
uavParam.aero.Cnb=0.09180;
uavParam.aero.Cnp=-0.13267 ;
uavParam.aero.Cnr= -0.08829;
uavParam.aero.CnDa=  0.0049274;
uavParam.aero.CnDr=-0.067036;
uavParam.aero.CnDr=-0.067036;
%Fuselage
uavParam.aero.dragCoeffMov=0.027;

%% Define Geometry parameters
uavParam.geom.b=2;
uavParam.geom.AR=7.2727;
uavParam.geom.c=0.275;
uavParam.geom.e=0.986;
uavParam.geom.RotorArm1=[ 0.375 0.375 0];
uavParam.geom.RotorArm2=[ 0.375 -0.375 0];
uavParam.geom.RotorArm3=[-0.375 -0.375 0];
uavParam.geom.RotorArm4=[-0.375 0.375 0];             
uavParam.geom.Ixx=0.089*3;
uavParam.geom.Iyy=uavParam.geom.Ixx;
uavParam.geom.Izz=0.125*3;
uavParam.geom.mass=6.023;
uavParam.geom.PropDiameter=0.3052;
uavParam.motor.RPMMAX=10000;
uavParam.motor.tilt_trim=0;
uavParam.aero.dragCoeffMov=0.027;
uavParam.aero.g=9.806;
end
