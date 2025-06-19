function quadrotor_6dof_dynamics(block)
% level-2 MATLAB file S-Function

  setup(block);

%endfunction

function setup(block)

    block.NumDialogPrms     =   1;

    block.NumInputPorts     =   1;
    block.NumOutputPorts    =   1;

    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    block.InputPort(1).Dimensions   =   4;
    block.OutputPort(1).Dimensions  =   12;

    block.NumContStates = 12;
    block.NumDiscStates = 0;
    block.SampleTimes = [0 0];  % continuous-time

    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Derivatives',          @Derivatives);
    block.RegBlockMethod('Outputs',              @Outputs);

%endfunction

function Derivatives(block)
    % --- Load parameters ---
    param = block.DialogPrm(1).Data;
    I = block.DialogPrm(2).Data;

    m = param.mass;
    g = param.gravity;
    Ixx = I(1,1);
    Iyy = I(2,2);
    Izz = I(3,3);

    % --- Extract states ---
    x     = block.ContStates.Data(1);
    y     = block.ContStates.Data(2);
    z     = block.ContStates.Data(3);
    u     = block.ContStates.Data(4);
    v     = block.ContStates.Data(5);
    w     = block.ContStates.Data(6);
    phi   = block.ContStates.Data(7);
    theta = block.ContStates.Data(8);
    psi   = block.ContStates.Data(9);
    p     = block.ContStates.Data(10);
    q     = block.ContStates.Data(11);
    r     = block.ContStates.Data(12);

    % --- Inputs ---
    U = block.InputPort(1).Data;
    T = U(1);
    tau_phi   = U(2);
    tau_theta = U(3);
    tau_psi   = U(4);

    % Rotation matrix (body to inertial)
    R = [cos(theta)*cos(psi), ...
         sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
         cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
         cos(theta)*sin(psi), ...
         sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
         cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
        -sin(theta), ...
         sin(phi)*cos(theta), ...
         cos(phi)*cos(theta)];

    % Inertial velocity
    dpos = R * [u; v; w];

    % Translational dynamics in body frame
    du = r*v - q*w + 0;              % + fx/m if drag
    dv = p*w - r*u + 0;              % + fy/m if drag
    dw = q*u - p*v - g + T/m;

    % Euler angle rates
    E = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];
    dAngles = E * [p; q; r];

    % Angular dynamics (Euler)
    dp = (tau_phi + (Iyy - Izz)*q*r) / Ixx;
    dq = (tau_theta + (Izz - Ixx)*p*r) / Iyy;
    dr = (tau_psi + (Ixx - Iyy)*p*q) / Izz;

    % Output derivatives
    block.Derivatives.Data = [dpos; du; dv; dw; dAngles; dp; dq; dr];

%endfunction


function InitConditions(block)

  %% Initialize Dwork
  block.Dwork(1).Data = 0;
  
%endfunction

function Output(block)

  block.OutputPort(1).Data = block.Dwork(1).Data;
  
  %% Set the next hit for this block 
  block.NextTimeHit = block.CurrentTime + block.InputPort(1).Data(2);
  
%endfunction