%% Linearization of drone dynamics about hover & full-state feedback design
% ===============================
% AUTHOR Fabian Riether
% CREATE DATE 2013/08/25
% PURPOSE This code assists in linearizing drone dynamics and designing
% full-state feedback controls
% SPECIAL NOTES
% ===============================
%  2015/08/25 created
% ==================================
% ===============================
%  2017/02/15 Modifed by FB for ME231b class 
% ==================================
% ===============================
%  2017/08/29 Modifed by Nathan Lambert for Ionocraft Flight 
% ==================================

%% Load drone parameters from RoboticsToolbox
% load('parameters.mat');

%% State Variables: Equilibrium, desired, inputs
%% 
% Do not change, These are the location where linearization is calculated from -------------
equil_X = 0.0;      %[m], equilibrium X;
equil_Y = 0.0;      %[m], equilibrium Y;
equil_Z = 0.0;      %[m], equilibrium Z;
equil_yaw = 0 * pi/180; %[rad], equilibrium yaw;
% ------------------------------------------------------------------------------------------

%Ionocraft Mass and gravity
quad = struct('g',g,'M',m);

% Ionocraft Drag
% Drag = struct('Taux',Taux_drag,'Tauy',Tauy_drag,'Tauz', Tauz_drag,'Fxy',Fxy_drag,'Fz',Fz_drag);


%-----------
state_equil = [equil_X; equil_Y; equil_Z; equil_yaw; 0; 0; 0; 0; 0; 0; 0; 0]; %x_eq

input_equil = [quad.g*quad.M/cos(angle) ;0 ;0 ;0];                %u_eq
equil       = [state_equil; input_equil];

% Hard codes desired states to try
state_des = [0; 0; .022; 0; .00; .00; 0; 0; 0; 0; 0; 0]; %x_eq
% NOTE: The system will not converge with a nonzero roll or pitch desired
% state. Have not tested angular or linear velocities. For now, desired
% state sould be a vector of the form [X,Y,Z,YAW] with 0's appended as need

%% 
XYZ_initial_condition = [0.0; 0.0; 0.0]; %[m], initial XYZ position

%% LQR Defintions
% Remember, LQR is ~ argmin{xQx + uRu}
% So, Q penalizes states and R penalizes inputs
% higher values = stronger penalty from desires
   
 
Q = 10*...
    [10 0 0 0 0 0 0 0 0 0 0 0; ...  X
     0 10 0 0 0 0 0 0 0 0 0 0; ...  Y
     0 0 100 0 0 0 0 0 0 0 0 0; ...  Z
     0 0 0 1 0 0 0 0 0 0 0 0; ...   YAW
     0 0 0 0 .1 0 0 0 0 0 0 0; ...   PITCH
     0 0 0 0 0 .1 0 0 0 0 0 0; ...   ROLL
     0 0 0 0 0 0 .001 0 0 0 0 0; ...   Vx
     0 0 0 0 0 0 0 .001 0 0 0 0; ...   Vy
     0 0 0 0 0 0 0 0 .001 0 0 0; ...   Vz
     0 0 0 0 0 0 0 0 0 .005 0 0; ...   p
     0 0 0 0 0 0 0 0 0 0 .005 0; ...   q
     0 0 0 0 0 0 0 0 0 0 0 .005];      
        
R = 1e8*...
    [1 0 0 0; ...
     0 1 0 0; ...
     0 0 1 0; ...
     0 0 0 1];


%% 1.1) Simplified Dynamics

%symbolic variables
syms Pxw Pyw Pzw yaw pitch roll dpx dpy dpz p q r T tauy taup taur;
symsvector  = [Pxw; Pyw; Pzw ;yaw ;pitch ;roll ;dpx ;dpy ;dpz ;p ;q ;r ;T ;tauy ;taup ;taur];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Notation used in ME231b, 3D dynamics
%symsvector  = [X,Y,Z, yaw (psi) ;pitch (theta) ;roll (phi) ;
%               vx ;vy ;vz ;wx ;wy ;wz ;T ;tau_z ;tau_y ;tau_x];


%Inertia
J            = I_B;

%Define Rotation matrices
Ryaw = [
	[ cos(yaw), -sin(yaw), 0],
	[ sin(yaw),  cos(yaw), 0],
	[        0,         0, 1]
];

Rpitch = [
	[  cos(pitch), 0, sin(pitch)],
	[           0, 1,          0],
	[ -sin(pitch), 0, cos(pitch)]
];

Rroll = [
	[ 1,         0,          0],
	[ 0, cos(roll), -sin(roll)],
	[ 0, sin(roll),  cos(roll)]
];

Body2Global = Ryaw*Rpitch*Rroll;
Global2Body = simplify(Body2Global^-1);

%Transformation from body rates p-q-r to euler rates yaw pitch roll
iW = ...
    [0        sin(roll)          cos(roll);             
     0        cos(roll)*cos(pitch) -sin(roll)*cos(pitch);
     cos(pitch) sin(roll)*sin(pitch) cos(roll)*sin(pitch)] / cos(pitch);



%% Dynamics
%----------      
%P dot      
P_dot           = simplify(Body2Global*[dpx;dpy;dpz]);
P_dot_jacobian  = jacobian(P_dot,symsvector);
P_dot_jacobian_eql = subs(P_dot_jacobian,symsvector,equil);

%O dot      
O_dot           = iW*[p;q;r];
O_dot_jacobian  = jacobian(O_dot,symsvector);
O_dot_jacobian_eql = subs(O_dot_jacobian,symsvector,equil);

%p ddot      
p_ddot          = Global2Body*[0;0;-quad.g] + T/quad.M*[0;0;1] -cross(transpose([p,q,r]),transpose([dpx,dpy,dpz]));
p_ddot_jacobian = jacobian(p_ddot,symsvector);
p_ddot_jacobian_eql = subs(p_ddot_jacobian,symsvector,equil);

%o ddot      
o_ddot          = inv(J)*([taur; taup; tauy] - cross([p;q;r],J*[p;q;r]));
o_ddot_jacobian = jacobian(o_ddot,symsvector);
o_ddot_jacobian_eql = subs(o_ddot_jacobian,symsvector,equil);

%Dynamics matrix
%---------- 

matrixAB = [P_dot_jacobian_eql;O_dot_jacobian_eql;p_ddot_jacobian_eql;o_ddot_jacobian_eql];
A = double(matrixAB(1:12,1:12));
B = double(matrixAB(1:12,11:16));   %11 was a 13, 31 Aug 2017


% ############## convert thruster inputs to T, Taux, Tauy, and Tauz #########
% ionocraft inputs are F1, F2, F3, F4
% [T; Tauz; Tauy; Taux;] = M * [F4; F3; F2; F1]
%
%        Top view of ionocraft
%            ____ ____            ^ x
%           |    |    |           | 
%           | F2 | F1 |           |
%           |____|____|    y <---(.)
%           |    |    |            z
%           | F3 | F4 |
%           |____|____|

 % set B equal to old B times M
 B = B*M2;
% ###########################################################################

% K = lqr(A,B,Q,R);
%Note x_nonlinearSys = x_eq + x_linearizedSys! Thus, x0_linearizedSys = x0_nonlinear - x_eq; 
%Note u_nonlinearSys = u_eq + x_linearizedSys!




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{





%% 1.2) Linearizing Full Nonlinear Simulink Model (the model from Robotics Toolbox)
%use Simulation/controllers/controller_fullstate/Poleplacement/linearizeDrone(...).slx and Simulink's ControlDesign/Linear Analysis

%% 2.1) Designing Full-state Feedback Controllers with Simplified Dynamics Model (1.1) via Pole Placement

%  %Note: We linearized about hover. This also implies: The control "policy"
%  %to correct a position error was derived under a yaw-angle of zero!
%  %If your drone yaw-drifts 90 deg and runs into a world-X-error, it will
%  %still believe that pitch is the right answer to correct for this position error! You can compensate for this by
%  %rotation the X-Y-error by the current yaw angle.
% 
% % Find states to decouple
% [V,J]   = jordan(A);
% Veig_nrm = diag(1./sum(V,1))*V; % decoupled system will have a new state-vector x_dec = inv(Veig_nrm)*x
% 
% % System matrices of decoupled system
% A_dec   = inv(Veig_nrm)*A*Veig_nrm;
% B_dec   = inv(Veig_nrm)*B;
% 
% % Define decoupled subsystems
% A_dec_x   = ...
% B_dec_x   = ...
% 
% A_dec_z   = ...
% B_dec_z   = ...
% 
% A_dec_y   = ...
% B_dec_y   = ...
% 
% A_dec_yaw = ...
% B_dec_yaw = ...
% 
% 
% % Compute decoupled  subsystems Transfer Function (TF)
% % TF from ... to x
% G_x = ...
% 
% % TF from ... to y
% G_y = ...
% 
% % TF from ... to z
% G_z = ...
% 
% % TF from ... to yaw
% G_yaw = ...
% 
% % Now place your own poles for the decoupled subsystems separately
% 
% xpoles      = [-9+6i;-9-6i;-0.18+1.8i;-0.18-1.8i];
% ypoles      = [-60;-4;-0.16+2i;-0.16-2i];       
% yawpoles    = [-3;-3.1];
% zpoles      = [-2;-2.1];               % Play around with poles here: Slow poles [-2;-2.1], Fast poles [-5;-5.1];
% %zpoles     = [-5;-5.1];               % Play around with poles here: Slow poles [-2;-2.1], Fast poles [-5;-5.1];
% 
% K_dec_x     = ...
% K_dec_z     = ...
% K_dec_y     = ...    
% K_dec_yaw   = ...
% 
% % Compute Full-state feedback for 'original' system
% K_poleplace = [K_dec_x K_dec_z K_dec_y K_dec_yaw]*inv(Veig_nrm);
% K_poleplace(abs(K_poleplace)<1e-7)=0;

%}
