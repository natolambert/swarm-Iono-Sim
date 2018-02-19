%% SETUP
close all

figure;
X = state_vector.Data(:,1)*m_to_cm;
Y = state_vector.Data(:,2)*m_to_cm;
Z = state_vector.Data(:,3)*m_to_cm;
psi = state_vector.Data(:,4);       % Yaw
theta = state_vector.Data(:,5);     % Pitch
phi = state_vector.Data(:,6);       % Roll



 %% Tools for plotting in global frame from data:


%% PLOTTING

%%%%%%%%%%%%%%% 3d visualization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(4,4,[1 2 5 6 9 10 13 14])

x_lim = [-10 30];       %[cm], x axis in 3D quiver plot
y_lim = [-30 10];       %[cm], y axis in 3D quiver plot
z_lim = [-5 5];         %[cm], z axis in 3D quiver plot
view_1 = 15;            %viewing angle for 3D quiver plot

set(gca,'XLim',x_lim,'YLim',y_lim,'ZLim',z_lim);

view_1 = 45;            %viewing angle for 3D quiver plot, first parameter (like yaw rotation of plot)
view_2 = 15;            %viewing angle for 3D quiver plot, second parameter (roll of plot)

view(view_1,view_2);
hold on;
grid on;
title('Simulated Flight (Takeoff)')
xlabel('X [cm]');
ylabel('Y [cm]');
zlabel('Z [cm]');

% plot X on xy plane
quiver3([0 0 0 0],[0 0 0 0],[0 0 0 0],[100 0 -100 0],[0 100 0 -100],[0 0 0 0],'ShowArrowHead','Off','Color','k');

scale_fact = 19500;
numpts = floor(length(X)/scale_fact);

% Add's trajectory lines to plot
plot3(X(1:scale_fact:end),Y(1:scale_fact:end),Z(1:scale_fact:end),'ro') 
plot3(X,Y,Z,'-k') 

% Plots atttitude's
for i=1:scale_fact:length(X)
%     loc = [X(i), Y(i), Z(i)];
%     
%     % Calculate needed global frame vector for 3 axes
%     axes_iono_x = b2g([.5; 0; 0],[psi(i),theta(i),phi(i)]);
%     axes_iono_y = b2g([0; .5; 0],[psi(i),theta(i),phi(i)]);
%     axes_iono_z = b2g([0; 0; .5],[psi(i),theta(i),phi(i)]);
%     
%     % Plots (can reduce this to one line)
%     quiver3(X(i), Y(i), Z(i), axes_iono_x(1), axes_iono_x(2), axes_iono_x(3),'g','linewidth',2)
%     quiver3(X(i), Y(i), Z(i), axes_iono_y(1), axes_iono_y(2), axes_iono_y(3),'r','linewidth',2)
%     quiver3(X(i), Y(i), Z(i), axes_iono_z(1), axes_iono_z(2), axes_iono_z(3),'b','linewidth',2)
%     
    yaw = psi(i);
    pitch = theta(i);
    roll = phi(i);
    ionocraft_length = 1; %[cm], length of ionocraft in simulation

    %% Craft

    %compute ionocraft length vectors in the global frame
    x_vec_global_pos = b2g([ionocraft_length; 0; 0],[yaw,pitch,roll]);
    x_vec_global_neg = b2g([-ionocraft_length; 0; 0],[yaw,pitch,roll]);
    y_vec_global_pos = b2g([0; ionocraft_length; 0],[yaw,pitch,roll]);
    y_vec_global_neg = b2g([0; -ionocraft_length; 0],[yaw,pitch,roll]);

    %plot four main ionocraft length vectors in the global frame
    x_vec_global_pos_head = quiver3(X(i),Y(i),Z(i),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','r');
    x_vec_global_neg_head = quiver3(X(i),Y(i),Z(i),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_pos_head = quiver3(X(i),Y(i),Z(i),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_neg_head = quiver3(X(i),Y(i),Z(i),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

    %plot four offset x body frame vectors in the inertial frame
    x_vec_global_pos_offset_1_head = quiver3(X(i) + y_vec_global_pos(1),Y(i) + y_vec_global_pos(2),Z(i) + y_vec_global_pos(3),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    x_vec_global_pos_offset_2_head = quiver3(X(i) + y_vec_global_neg(1),Y(i) + y_vec_global_neg(2),Z(i) + y_vec_global_neg(3),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    x_vec_global_neg_offset_1_head = quiver3(X(i) + y_vec_global_pos(1),Y(i) + y_vec_global_pos(2),Z(i) + y_vec_global_pos(3),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
    x_vec_global_neg_offset_2_head = quiver3(X(i) + y_vec_global_neg(1),Y(i) + y_vec_global_neg(2),Z(i) + y_vec_global_neg(3),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

    %plot four offset y body frame vectors in the inertial frame
    y_vec_global_pos_offset_1_head = quiver3(X(i) + x_vec_global_pos(1),Y(i) + x_vec_global_pos(2),Z(i) + x_vec_global_pos(3),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_pos_offset_2_head = quiver3(X(i) + x_vec_global_neg(1),Y(i) + x_vec_global_neg(2),Z(i) + x_vec_global_neg(3),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_neg_offset_1_head = quiver3(X(i) + x_vec_global_pos(1),Y(i) + x_vec_global_pos(2),Z(i) + x_vec_global_pos(3),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_neg_offset_2_head = quiver3(X(i) + x_vec_global_neg(1),Y(i) + x_vec_global_neg(2),Z(i) + x_vec_global_neg(3),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

end
text(X(20:scale_fact:end),Y(1:scale_fact:end),Z(1:scale_fact:end),...
       strcat("             t =",num2str(state_vector.Time(1:scale_fact:end))," s"),'fontsize',18) 


   
%{
        yaw = psi(i);
        pitch = theta(i);
        roll = phi(i);
        ionocraft_length = 1; %[cm], length of ionocraft in simulation

        %% Craft
        
        %compute ionocraft length vectors in the global frame
        x_vec_global_pos = b2g([ionocraft_length; 0; 0],[yaw,pitch,roll]);
        x_vec_global_neg = b2g([-ionocraft_length; 0; 0],[yaw,pitch,roll]);
        y_vec_global_pos = b2g([0; ionocraft_length; 0],[yaw,pitch,roll]);
        y_vec_global_neg = b2g([0; -ionocraft_length; 0],[yaw,pitch,roll]);

        %plot four main ionocraft length vectors in the global frame
        x_vec_global_pos_head = quiver3(X(i),Y(i),Z(i),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','r');
        x_vec_global_neg_head = quiver3(X(i),Y(i),Z(i),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
        y_vec_global_pos_head = quiver3(X(i),Y(i),Z(i),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
        y_vec_global_neg_head = quiver3(X(i),Y(i),Z(i),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

        %plot four offset x body frame vectors in the inertial frame
        x_vec_global_pos_offset_1_head = quiver3(X(i) + y_vec_global_pos(1),Y(i) + y_vec_global_pos(2),Z(i) + y_vec_global_pos(3),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
        x_vec_global_pos_offset_2_head = quiver3(X(i) + y_vec_global_neg(1),Y(i) + y_vec_global_neg(2),Z(i) + y_vec_global_neg(3),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
        x_vec_global_neg_offset_1_head = quiver3(X(i) + y_vec_global_pos(1),Y(i) + y_vec_global_pos(2),Z(i) + y_vec_global_pos(3),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
        x_vec_global_neg_offset_2_head = quiver3(X(i) + y_vec_global_neg(1),Y(i) + y_vec_global_neg(2),Z(i) + y_vec_global_neg(3),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

        %plot four offset y body frame vectors in the inertial frame
        y_vec_global_pos_offset_1_head = quiver3(X(i) + x_vec_global_pos(1),Y(i) + x_vec_global_pos(2),Z(i) + x_vec_global_pos(3),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
        y_vec_global_pos_offset_2_head = quiver3(X(i) + x_vec_global_neg(1),Y(i) + x_vec_global_neg(2),Z(i) + x_vec_global_neg(3),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
        y_vec_global_neg_offset_1_head = quiver3(X(i) + x_vec_global_pos(1),Y(i) + x_vec_global_pos(2),Z(i) + x_vec_global_pos(3),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
        y_vec_global_neg_offset_2_head = quiver3(X(i) + x_vec_global_neg(1),Y(i) + x_vec_global_neg(2),Z(i) + x_vec_global_neg(3),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

       %% Last Frame
    
    %plot four main ionocraft length vectors in the global frame
    x_vec_global_pos_head = quiver3(X(i),Y(i),Z(i),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','r');
    x_vec_global_neg_head = quiver3(X(i),Y(i),Z(i),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_pos_head = quiver3(X(i),Y(i),Z(i),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_neg_head = quiver3(X(i),Y(i),Z(i),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

    %plot four offset x body framvectors in the inertial frame
    x_vec_global_pos_offset_1_head = quiver3(X(i) + y_vec_global_pos(1),Y(i) + y_vec_global_pos(2),Z(i) + y_vec_global_pos(3),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    x_vec_global_pos_offset_2_head = quiver3(X(i) + y_vec_global_neg(1),Y(i) + y_vec_global_neg(2),Z(i) + y_vec_global_neg(3),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    x_vec_global_neg_offset_1_head = quiver3(X(i) + y_vec_global_pos(1),Y(i) + y_vec_global_pos(2),Z(i) + y_vec_global_pos(3),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
    x_vec_global_neg_offset_2_head = quiver3(X(i) + y_vec_global_neg(1),Y(i) + y_vec_global_neg(2),Z(i) + y_vec_global_neg(3),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

    %plot four offset y body frame vectors in the inertial frame
    y_vec_global_pos_offset_1_head = quiver3(X(i) + x_vec_global_pos(1),Y(i) + x_vec_global_pos(2),Z(i) + x_vec_global_pos(3),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_pos_offset_2_head = quiver3(X(i) + x_vec_global_neg(1),Y(i) + x_vec_global_neg(2),Z(i) + x_vec_global_neg(3),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_neg_offset_1_head = quiver3(X(i) + x_vec_global_pos(1),Y(i) + x_vec_global_pos(2),Z(i) + x_vec_global_pos(3),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_neg_offset_2_head = quiver3(X(i) + x_vec_global_neg(1),Y(i) + x_vec_global_neg(2),Z(i) + x_vec_global_neg(3),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

    %plot four main ionocraft length vectors projected onto the xy plane of the global frame
    x_vec_global_pos_yaw_head = quiver3(X(i),Y(i),0,x_vec_global_pos(1),x_vec_global_pos(2),0,'ShowArrowHead','Off','Color','r');
    x_vec_global_neg_yaw_head = quiver3(X(i),Y(i),0,x_vec_global_neg(1),x_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');
    y_vec_global_pos_yaw_head = quiver3(X(i),Y(i),0,y_vec_global_pos(1),y_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
    y_vec_global_neg_yaw_head = quiver3(X(i),Y(i),0,y_vec_global_neg(1),y_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');

    %plot four offset x body frame vectors in the xy plane of the global frame
    x_vec_global_pos_xy_offset_1_head = quiver3(X(i) + y_vec_global_pos(1),Y(i) + y_vec_global_pos(2),0,x_vec_global_pos(1),x_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
    x_vec_global_pos_xy_offset_2_head = quiver3(X(i) + y_vec_global_neg(1),Y(i) + y_vec_global_neg(2),0,x_vec_global_pos(1),x_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
    x_vec_global_neg_xy_offset_1_head = quiver3(X(i) + y_vec_global_pos(1),Y(i) + y_vec_global_pos(2),0,x_vec_global_neg(1),x_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');
    x_vec_global_neg_xy_offset_2_head = quiver3(X(i) + y_vec_global_neg(1),Y(i) + y_vec_global_neg(2),0,x_vec_global_neg(1),x_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');

    %plot four offset y body frame vectors in the xy plane of the global frame
    y_vec_global_pos_xy_offset_1_head = quiver3(X(i) + x_vec_global_pos(1),Y(i) + x_vec_global_pos(2),0,y_vec_global_pos(1),y_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
    y_vec_global_pos_xy_offset_2_head = quiver3(X(i) + x_vec_global_neg(1),Y(i) + x_vec_global_neg(2),0,y_vec_global_pos(1),y_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
    y_vec_global_neg_xy_offset_1_head = quiver3(X(i) + x_vec_global_pos(1),Y(i) + x_vec_global_pos(2),0,y_vec_global_neg(1),y_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');
    y_vec_global_neg_xy_offset_2_head = quiver3(X(i) + x_vec_global_neg(1),Y(i) + x_vec_global_neg(2),0,y_vec_global_neg(1),y_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');

%}
hold off

%%%%%%%%%%%%%%% Force Curves %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t_lim = 1;      % limit for force and attitude plots

subplot(4,4,[11 12 15 16])
% yyaxis right
% ylabel('Controller Voltage (kV)')
% plot(force_inputs.Time,sin(force_inputs.Data(:,1).^2));
% yyaxis left
plot(force_inputs.Time,force_inputs.Data(:,1) * N_to_uN);
ylabel('Thruster Force [uN]');
title('F4');
xlabel(xlabel_Time);
xlim([0 t_lim])
% 
% subplot(4,3,5)
% yyaxis right
% ylabel('Controller Voltage (kV)')
% plot(force_inputs.Time,sin(force_inputs.Data(:,2).^2));
% yyaxis left
% plot(force_inputs.Time,force_inputs.Data(:,2) * N_to_uN);
% ylabel('Thruster Force [uN]');
% title('F3');
% xlabel(xlabel_Time);
% xlim([0 t_lim])
% 
% subplot(4,3,8)
% yyaxis right
% ylabel('Controller Voltage (kV)')
% plot(force_inputs.Time,sin(force_inputs.Data(:,3).^2));
% yyaxis left
% plot(force_inputs.Time,force_inputs.Data(:,3) * N_to_uN);
% ylabel('Thruster Force [uN]');
% title('F2');
% xlabel(xlabel_Time);
% xlim([0 t_lim])
% 
% subplot(4,3,11)
% yyaxis right
% ylabel('Controller Voltage (kV)')
% plot(force_inputs.Time,sin(force_inputs.Data(:,4).^2));
% yyaxis left
% plot(force_inputs.Time,force_inputs.Data(:,4) * N_to_uN);
% ylabel('Thruster Force [uN]');
% title('F1');
% xlabel(xlabel_Time);
% xlim([0 t_lim])

%%%%%%%%%%%%%%% Control Ability %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(4,4,3)
hold on
plot(state_vector.Time,estimates.Data(:,3), 'color',[0.9100    0.4100    0.1700]);
plot(state_vector.Time,accel_z.Data(:,1), 'b');
ylabel('Z Accel [m/s^2]');
title('Z Accel');
legend('Noisy Data', 'Ground Truth')
xlim([0 t_lim])
hold off

subplot(4,4,4)
plot(state_vector.Time,state_vector.Data(:,3) * m_to_cm);
ylabel('Z [cm]');
title('Z');
xlim([0 t_lim])
hold off

subplot(4,4,7)
hold on
plot(state_vector.Time,estimates.Data(:,5) * rad_to_deg, 'color',[0.9100    0.4100    0.1700]);
plot(state_vector.Time,state_vector.Data(:,5) * rad_to_deg, 'b');
ylabel('Pitch [deg]');
title('Pitch');
xlim([0 t_lim])
hold off

subplot(4,4,8)
hold on
plot(state_vector.Time,estimates.Data(:,6) * rad_to_deg, 'color',[0.9100    0.4100    0.1700]);
plot(state_vector.Time,state_vector.Data(:,6) * rad_to_deg, 'b');
ylabel('Roll [deg]');
title('Roll');
xlim([0 t_lim])
hold off



%%
function out = b2g(vect, att)
    % Define Rotation matrices of given index
    % vect = [x,y,z], att = [yaw,pitch,roll]
    yaw = att(1);
    pitch = att(2);
    roll = att(3);
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
    out = Body2Global*vect;
end