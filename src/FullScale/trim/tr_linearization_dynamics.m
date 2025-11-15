function tr_linearization_dynamics(block)
%.. level-2 s-function to linearize fixed-wing model
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
    block.ContStates.Data(1)    =   1.0;

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
    vt      =   X(1);
    alpha   =   X(2);
    beta    =   X(3);
    phi     =   X(4);
    theta   =   X(5);
    psi     =   X(6);        
    p       =   X(7);
    q       =   X(8);
    r       =   X(9);   

    %.. set up initial outputs
    block.Derivatives.Data  =   zeros(12,1);

    %.. velocity component in body frame
    u       =   vt*cos(alpha)*cos(beta);
    v       =   vt*sin(beta);
    w       =   vt*sin(alpha)*cos(beta);
        
    %.. translational dynamics
    uvwdot  =   -cross([p; q; r],[u; v; w])+[Fx; Fy; Fz]/mass;
    udot    =   uvwdot(1);
    vdot    =   uvwdot(2);
    wdot    =   uvwdot(3);
        
    %.. differential equation reprented in vt, alpha, beta        
    block.Derivatives.Data(1)   =   (u*udot+v*vdot+w*wdot)/vt;
    block.Derivatives.Data(2)   =   (u*wdot-w*udot)/(u^2+w^2);
    block.Derivatives.Data(3)   =   (vt*vdot-v*(u*udot+v*vdot+w*wdot)/vt)/(vt*sqrt(u^2+w^2));    
        
    %.. rotation kinematics
    block.Derivatives.Data(4)   =   p+tan(theta)*(q*sin(phi)+r*cos(phi));
    block.Derivatives.Data(5)   =   q*cos(phi)-r*sin(phi);
    block.Derivatives.Data(6)   =   (q*sin(phi)+r*cos(phi))/cos(theta);     

    %.. moment equations
    block.Derivatives.Data(7)   =   ((Ixx-Iyy+Izz)*Ixz)/(Ixx*Izz-Ixz*Ixz)*p*q-...
                                    (Izz*(Izz-Iyy)+Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz)*q*r+...
                                    Izz/(Ixx*Izz-Ixz*Ixz)*Mx+...
                                    Ixz/(Ixx*Izz-Ixz*Ixz)*Mz;
    block.Derivatives.Data(8)   =   (Izz-Ixx)/Iyy*p*r-Ixz/Iyy*(p^2-r^2)+1/Iyy*My;
    block.Derivatives.Data(9)   =   -((Ixx-Iyy+Izz)*Ixz)/(Ixx*Izz-Ixz*Ixz)*r*q+...
                                    (Ixx*(Ixx-Iyy)+Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz)*p*q+...
                                    Ixz/(Ixx*Izz-Ixz*Ixz)*Mx+...
                                    Ixx/(Ixx*Izz-Ixz*Ixz)*Mz;
        
    %.. translational kinematics
    block.Derivatives.Data(10)  =   cos(theta)*cos(psi)*u+...
                                    (-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi))*v+...
                                    (sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi))*w;
    block.Derivatives.Data(11)  =   cos(theta)*sin(psi)*u+...
                                    (cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi))*v+...
                                    (-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi))*w;
    block.Derivatives.Data(12)  =   -sin(theta)*u+...
                                    sin(phi)*cos(theta)*v+...
                                    cos(phi)*cos(theta)*w;

%endfunction

function Outputs(block)
  
  block.OutputPort(1).Data = block.ContStates.Data;
  
%endfunction