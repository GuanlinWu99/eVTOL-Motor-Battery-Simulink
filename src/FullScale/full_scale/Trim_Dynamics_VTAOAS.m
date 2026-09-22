%.. m-file S-function
function [sys, X0, str, ts] = Trim_Dynamics_VTAOAS(t, X, U, flag)

%.. Global Variables
    global 	g_const	Mass    I_xx    I_yy    I_zz    I_xz
	global  VT_0    Gamma_trim  Alpha_trim  Theta_dot_trim  Turn_dot_trim   
    
    
%-------------------------< Satate Derivatives >--------------------------%
    if flag == 1

    %.. Input Variables
    %.. Total Forces
        X_total     =   U(1);
        Y_total     =   U(2);
        Z_total     =   U(3);
        
    %.. Total Moments    
        L_total     =   U(4);
        M_total     =   U(5);
        N_total     =   U(6);        
        
    %.. State Variables
        VT          =   X(1);
        Alpha       =   X(2);
        Beta        =   X(3);
        P           =   X(4);
        Q           =   X(5);
        R           =   X(6);
        Phi         =   X(7);
        Theta       =   X(8);
        Psi         =   X(9);        
                
    %.. Velocity Components in Body Axis
        U_body      =   VT*cos(Alpha)*cos(Beta);
        V_body      =   VT*sin(Beta);
        W_body      =   VT*sin(Alpha)*cos(Beta);
        
    %.. Forces Equations        
        U_dot       =   R*V_body-Q*W_body+(X_total/Mass);
        V_dot       =   -R*U_body+P*W_body+(Y_total/Mass);
        W_dot       =   Q*U_body-P*W_body+(Z_total/Mass);        
        
    %.. Differential Equation of VT, AOA, AOS        
        DUM         =   (U_body^2+W_body^2);
        
        XD(1)       =   (U_body*U_dot + V_body*V_dot + W_body*W_dot)/VT;
        XD(2)       =   (U_body*W_dot - W_body*U_dot)/DUM;
        XD(3)       =   (VT*V_dot- V_body*XD(1))/(cos(Beta)*VT^2);    
        
    %.. Miscellaneous Constants for Moment Equations
        GAM         =   I_xx*I_zz-I_xz*I_xz;
        C1          =   ((I_yy-I_zz)*I_zz-I_xz*I_xz)/GAM;
        C2          =   ((I_xx-I_yy+I_zz)*I_xz)/GAM;
        C3          =   I_zz/GAM;
        C4          =   I_xz/GAM;
        C5          =   (I_zz-I_xx)/I_yy;
        C6          =   I_xz/I_yy; 
        C7          =   1/I_yy;
        C8          =   (I_xx*(I_xx-I_yy)+I_xz*I_xz)/GAM;
        C9          =   I_xx/GAM; 
        
    %.. Moment Equations        
        XD(4)       =   (C1*R+C2*P)*Q+C3*L_total+C4*N_total;
        XD(5)       =   C5*P*R-C6*(P*P-R*R)+C7*M_total;
        XD(6)       =   (C8*P-C2*R)*Q+C4*L_total+C9*N_total;
       
    %.. Kinematic Equations
        XD(7)       =   P+tan(Theta)*(Q*sin(Phi)+R*cos(Phi));
        XD(8)       =   Q*cos(Phi)-R*sin(Phi);
        XD(9)       =   (Q*sin(Phi)+R*cos(Phi))/cos(Theta);       
        
    %.. Constraints for Trim Calculation
    %.. Pull-up Constraint 
        XD(8)       =   XD(8)-  ;
    
    %..	Rate-Of-Climb Constraint
        sing        =   sin(Gamma_trim);
        a1          =   cos(Alpha)*cos(Beta);
        b1          =   sin(Phi)*sin(Beta)+cos(Phi)*sin(Alpha)*cos(Beta);
        ROC         =   sing-a1*sin(Theta)+b1*cos(Theta);
    
    %.. Coordinate Turn Constraint
        XD(9)       =   XD(9)-Turn_dot_trim;
        G_turn      =   Turn_dot_trim*VT_0/g_const;
        CTC         =   sin(Phi)-G_turn*cos(Beta)*(sin(Alpha)*tan(Theta)+cos(Alpha)*cos(Phi));
        
    %   Trim Constraints        
        XD(10)      =   VT-VT_0;
        XD(11)      =   ROC;
        XD(12)      =   CTC;        
        
        sys         =   XD;

%-------------------------------------------------------------------------%
  

%-------------------------< Output Calculation >--------------------------% 
    elseif flag == 3
        
    %.. Output Variable
        sys(1)      = 	X(1);                                               % VT (m/s)
        sys(2)      =   X(2);                                               % Alpha (rad)
        sys(3)      =   X(3);                                               % Beta (rad)
        sys(4)      =   X(4);                                               % P (rad/s)
        sys(5)      =   X(5);                                               % Q (rad/s)
        sys(6)      =   X(6);                                               % R (rad/s)
        sys(7)      =   X(7);                                               % Phi (rad)
        sys(8)      =   X(8);                                               % Theta (rad)
        sys(9)      =   X(9);                                               % Psi (rad)
        sys(10)     =   X(10);                                              % Velocity (Constraint)
        sys(11)     =   X(11);                                              % Rate-of-climb (Constraint)
        sys(12)     =   X(12);                                              % Coordinate turn (Constraint)
%-------------------------------------------------------------------------%


%---------------< System Information & Initial Conditions >---------------%
    elseif flag == 0
        
    %.. S-function Setting
        sizes                =  simsizes;
        sizes.NumContStates  =	12;
        sizes.NumDiscStates  =	0;
        sizes.NumOutputs     =	12;
        sizes.NumInputs      =	6;
        sizes.DirFeedthrough =	0;
        sizes.NumSampleTimes =	1;
        
        sys = simsizes(sizes);
        str = [ ];
        ts  = [0, 0];

    %.. Initial Conditions of State Variables
        X0 = zeros(12,1);
        
    else
        
        sys = [ ];
        
    end
%-------------------------------------------------------------------------%
