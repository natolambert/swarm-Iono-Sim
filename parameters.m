% Ionocraft parameters
% North-West-Up convention is used. North = x, East = y, Up = z.
% Updated by Nathan Lambert nol@berkeley.edu Aug 2017


%% Clean up
clc;
clear all;
close all;

% Plot Tweaks
set(0,'defaultAxesFontSize',15)
set(0,'DefaultLineLineWidth',2)

%% Basic and Physical Paramters
g = 9.8;        % [m/s^2], acceleration of gravity
m = 60e-6;      % [kg], body mass  Changed from 10e-6 9/5/2017    5-e-6 is with IMU + flexboard
lx = 1e-2;      % [m], x distance to body center of mass
ly = 1e-2;      % [m], y distance to body center of mass
lz = 20e-6;     % [m], z distance to body center of mass
rho = 1.01;     % [kg/m^3] 

I_B_x = (1/12)*m*ly^2;          %[kg*m^2], moment of inertia around x axis
I_B_y = (1/12)*m*lx^2;          %[kg*m^2], moment of inertia around y axis
I_B_z = (1/12)*m*(lx^2 + ly^2); %[kg*m^2], moment of inertia around z axis
I_B = [I_B_x 0 0; 0 I_B_y 0; 0 0 I_B_z]; % [kg*m^2], moment of inertia matrix

drag_oom = 1e-4;
bx = .05*drag_oom; %[Ns/m], damping coefficient
by = .05*drag_oom; %[Ns/m], damping coefficient
bz = 3*drag_oom; %[Ns/m], damping coefficient, 0.4e-7 used for doing roll flip
btaux = .5*drag_oom; %[Ns/m], damping coefficient, 0.4e-7 used for doing roll flip
btauy = .5*drag_oom; %[Ns/m], damping coefficient
btauz = .001*drag_oom; %[Ns/m], damping coefficient


%% Drag
%{
% Fit drag coeff data

fitted = fittype('a+d*exp(-c*(x-b)-e*x)');
fitted = fittype('a+b/(x-c)');
v = linspace(1,12,25);
Cd = (1/2).*[66, 13, 7, 5.4, 4.6, 4.1, 3.9, 3.8, 3.6, 3.5, 3.4, 3.3, 3.2, 3.15, 3.1 , 3, 2.95, 2.9, 2.8, 2.75, 2.7, 2.65, 2.6, 2.55 , 2.5  ];
[fit1, gof1, fitinf1] = fit(v',Cd',fitted, 'StartPoint', [2 6.1 -.1], 'TolFun', 1e-8);

% Preload drag numbers for normal velocities! 

% drag function to be integrated
fun_rot = @(x,a) 1./2.*fit1(a.*abs(x)).*rho.*(a.*x).^2.*abs(x);
fun_lin = @(v) 1./2.*fit1(v).*rho.*v.^2;


% Precomputes Values
w = linspace(0,10,500);
v2 = linspace(0,5,500);
for i = 1:500
    int = integral(@(x)fun_rot(x,w(i)),-ly,ly,'ArrayValued',true);
    Taux_drag(i) = 2*lx*int;
    Tauy_drag(i) = 2*ly*int;
    Tauz_drag(i) = 2*lz*int;
end

for j = 1:500
    Fxy_drag(j) = 4*lx*ly*fun_lin(v2(j));
    Fz_drag(j) = 4*lz*ly*fun_lin(v2(j));
end
%}
%% Adding curve fitting for ALL of the drag equations vs velocity / angular
%{
fitDrag = fittype('a*x^2 + b*x');

% 0 Taux

[fitTx, gofTx, fitinfoTx] = fit(w',Taux_drag',fitDrag, 'StartPoint', [1 1], 'TolFun', 1e-8);

% 1 Tauy

[fitTy, gofTy, fitinfoTy] = fit(w',Tauy_drag',fitDrag, 'StartPoint', [1 1], 'TolFun', 1e-8);

% 2 Tauz
[fitTz, gofTz, fitinfoTz] = fit(w',Tauz_drag',fitDrag, 'StartPoint', [1 1], 'TolFun', 1e-8);

% 3 Fxy
[fitFxy, gofFxy, fitinfoFxy] = fit(v2',Fxy_drag',fitDrag, 'StartPoint', [1 1], 'TolFun', 1e-8);

% 4 Fz
[fitFz, gofFz, fitinfoFz] = fit(v2',Fz_drag',fitDrag, 'StartPoint', [1 1], 'TolFun', 1e-8);

%} 
%{
fitTx = 

     General model:
     fitTx(x) = a*x^2 + b*x
     Coefficients (with 95% confidence bounds):
       a =    7.49e-10  (7.41e-10, 7.569e-10)
       b =   2.138e-09  (2.076e-09, 2.199e-09)


fitTy = 

     General model:
     fitTy(x) = a*x^2 + b*x
     Coefficients (with 95% confidence bounds):
       a =    7.49e-10  (7.41e-10, 7.569e-10)
       b =   2.138e-09  (2.076e-09, 2.199e-09)

fitTz = 

     General model:
     fitTz(x) = a*x^2 + b*x
     Coefficients (with 95% confidence bounds):
       a =   1.498e-12  (1.482e-12, 1.514e-12)
       b =   4.275e-12  (4.152e-12, 4.398e-12)


fitFxy = 

     General model:
     fitFxy(x) = a*x^2 + b*x
     Coefficients (with 95% confidence bounds):
       a =   0.0002158  (0.0002153, 0.0002162)
       b =   0.0005734  (0.0005717, 0.0005751)


fitFz = 

     General model:
     fitFz(x) = a*x^2 + b*x
     Coefficients (with 95% confidence bounds):
       a =   4.315e-07  (4.307e-07, 4.324e-07)
       b =   1.147e-06  (1.143e-06, 1.15e-06)
%}

% save('drag.mat', 'Taux_drag', 'Tauy_drag', 'Tauz_drag', 'Fxy_drag', 'Fz_drag')

%% Plotting thedrag such

% figure
% hold on
% plot(v,Cd)
% plot(fit1,'-.r')
% xlabel('linear velocity (m/s)')
% ylabel('Drag Coefficient (Cd)')
% title('Drag Coefficient Curve Fitting')
% legend('Raw Data', 'Fitted Curve')
% hold off
% 
% figure
% hold on
% plot(v2,Fxy_drag)
% xlabel('linear velocity (m/s)')
% ylabel('Drag Force (N)')
% title('Drag Force vs Linear Velocity (x)')
% hold off
% 
% figure
% hold on
% xlabel('angular velocity (rad/s)')
% ylabel('Drag Torque (N M)')
% plot(w,Taux_drag)
% title('Rotation Drag Torque vs Angular Vel (wx)')
% hold off
 
%% M Matrices 
c = 1e-2; % [m], coupling between thruster force and yaw torque bx = 1e-5; %[Ns/m], damping coefficient

% Standard Quadcopter 
M = ...     % [T; Tauz; Tauy; Taux;] = M * [F4; F3; F2; F1]
    [1   1   1  1;
     -c   c  -c  c;
     lx lx -lx -lx;
     -ly ly  ly -ly;];

angle = 0; 

% Ionocraft without XY thrusts - I think a couple of the +/-'s are off in
% this one. See below
M = ...     % [T; Tauz; Tauy; Taux;] = M * [F4; F3; F2; F1]
    [1*cos(angle)       1*cos(angle)    1*cos(angle)    1*cos(angle);
     -lx*sin(angle)     lx*sin(angle)   -lx*sin(angle)  lx*sin(angle);
     lx*cos(angle)      lx*cos(angle)   -lx*cos(angle)  -lx*cos(angle);
     -ly*cos(angle)     ly*cos(angle)   ly*cos(angle)   -ly*cos(angle);];
 
 % Ionocraft with XY thrusts
M2 = ...     % [Thrustx; Thrusty; Thrustz; Tauz; Tauy; Taux;] = M * [F4; F3; F2; F1]
    [0                  sin(angle)      0               -sin(angle);            % Thrustx - all will be +/-1*sin(angle)
     -sin(angle)        0               sin(angle)      0;                      % Thrusty
     1*cos(angle)       1*cos(angle)    1*cos(angle)    1*cos(angle);           % Thrustz
     -lx*sin(angle)     lx*sin(angle)   -lx*sin(angle)  lx*sin(angle);          % Tauz
     -lx*cos(angle)      -lx*cos(angle)   lx*cos(angle)  lx*cos(angle);         % Tauy
     ly*cos(angle)     -ly*cos(angle)   -ly*cos(angle)   ly*cos(angle);];       % Taux


%% Matrices for PID contorllers translating PID output to force inputs
M_z     = [1,1,1,1]';         % Translates PID of z direction uniformly across 4 thrusters
M_roll  = [1,-1,-1,1]';    
M_pitch = -1*[1,1,-1,-1]';


%% Plot Paramters (more in plot_simulation.m)
% x_lim = [-5 5];       %[cm], x axis in 3D quiver plot
% y_lim = [-5 5];       %[cm], y axis in 3D quiver plot
% z_lim = [0 10];         %[cm], z axis in 3D quiver plot
% view_1 = 15;            %viewing angle for 3D quiver plot, first parameter
% view_2 = 46;            %viewing angle for 3D quiver plot, second parameter

x_lim = [-10 30];       %[cm], x axis in 3D quiver plot
y_lim = [-30 10];       %[cm], y axis in 3D quiver plot
z_lim = [-5 5];         %[cm], z axis in 3D quiver plot
view_1 = 15;            %viewing angle for 3D quiver plot, first parameter
view_2 = 46;            %viewing angle for 3D quiver plot, second parameter


video_frame_frequency = 330; % how many frame jumps per frame that is recorded in video 66 = 60fps
sim_time = 1;          %[s], how long the simulation runs 
video_flag = 0;         % set to 1 to record video, set to 0 otherwise

save('parameters.mat');
