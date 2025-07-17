function vtol_linearization_dynamics(block)
%.. level-2 s-function to linearize vtol model
%.. setup method is used to setup the basic attributes of the
%.. s-function such as ports, parameters, etc
%.. do not add any other calls to the main body of the function

setup(block);

%endfunction

function setup(block)

    %.. register number of ports
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;
  
    %.. setup port properties to be inherited or dynamic
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    %. override input port properties
    block.InputPort(1).DatatypeID           =   0;
    block.InputPort(1).Complexity           =   'Real';
    block.InputPort(1).Dimensions           =   6;
    block.InputPort(1).DirectFeedthrough    =   false;

    %.. override output port properties
    block.OutputPort(1).DatatypeID          =   0; 
    block.OutputPort(1).Complexity          =   'Real';
    block.OutputPort(1).Dimensions          =   12;

    %.. register parameters
    block.NumDialogPrms     =   6;

    %.. register sample times
    block.SampleTimes       =   [0 0];
  
    %.. set number of continuous states
    block.NumContStates     =   12;

    %.. options
    %.. specify if accelerator should use TLC or call back into MATLAB file
    block.SetAccelRunOnTLC(false);
    
    %.. register methods
    block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Derivatives', @Derivatives);
    block.RegBlockMethod('Outputs', @Outputs);

%endfunction

function InitializeConditions(block)
    
    %.. initialize continuous states  
    block.ContStates.Data       =   zeros(12,1);

function Derivatives(block)

    %.. get trim conditions and parameters from dialog parameters
    mass            =   block.DialogPrm(1).Data;
    Ixx             =   block.DialogPrm(2).Data;
    Iyy             =   block.DialogPrm(3).Data;
    Izz             =   block.DialogPrm(4).Data;
    Ixz             =   block.DialogPrm(5).Data;
    g               =   block.DialogPrm(6).Data;

    %.. force and moment inputs: Fx, Fy, Fz, Mx, My, Mz
    U   =   block.InputPort(1).Data;
    Fx  =   U(1);
    Fy  =   U(2);
    Fz  =   U(3);
    Mx  =   U(4);
    My  =   U(5);
    Mz  =   U(6);

    %.. states
    X       =   block.ContStates.Data;
    x       =   X(1);
    y       =   X(2);
    z       =   X(3);
    u       =   X(4);
    v       =   X(5);
    w       =   X(6);
    phi     =   X(7);
    theta   =   X(8);
    psi     =   X(9);
    p       =   X(10);
    q       =   X(11);
    r       =   X(12);

    %.. set up initial outputs
    block.Derivatives.Data  =   zeros(12,1);

    %.. rotation matrix (body to inertial)
    R       =   [cos(theta)*cos(psi), ...
                 sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
                 cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                 cos(theta)*sin(psi), ...
                 sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
                 cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
                 -sin(theta), ...
                 sin(phi)*cos(theta), ...
                 cos(phi)*cos(theta)];

    %.. inertial kinematics
    block.Derivatives.Data(1:3) =   R*[u; v; w];

    %.. translational dynamics in body frame
    block.Derivatives.Data(4:6) =   -cross([p; q; r],[u; v; w])+[Fx; Fy; Fz]/mass;              

    %.. rotation kinematics
    block.Derivatives.Data(7)   =   p+tan(theta)*(q*sin(phi)+r*cos(phi));
    block.Derivatives.Data(8)   =   q*cos(phi)-r*sin(phi);
    block.Derivatives.Data(9)   =   (q*sin(phi)+r*cos(phi))/cos(theta); 

    % Angular dynamics (Euler)
    block.Derivatives.Data(10)  =   ((Ixx-Iyy+Izz)*Ixz)/(Ixx*Izz-Ixz*Ixz)*p*q-...
                                    (Izz*(Izz-Iyy)+Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz)*q*r+...
                                    Izz/(Ixx*Izz-Ixz*Ixz)*Mx+...
                                    Ixz/(Ixx*Izz-Ixz*Ixz)*Mz;
    block.Derivatives.Data(11)  =   (Izz-Ixx)/Iyy*p*r-Ixz/Iyy*(p^2-r^2)+1/Iyy*My;
    block.Derivatives.Data(12)  =   -((Ixx-Iyy+Izz)*Ixz)/(Ixx*Izz-Ixz*Ixz)*r*q+...
                                    (Ixx*(Ixx-Iyy)+Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz)*p*q+...
                                    Ixz/(Ixx*Izz-Ixz*Ixz)*Mx+...
                                    Ixx/(Ixx*Izz-Ixz*Ixz)*Mz;

%endfunction

function Outputs(block)

  block.OutputPort(1).Data = block.ContStates.Data;
  
%endfunction