%% Setup

clc;
close all
state_vector_rows = 4;
state_vector_cols = 3;
xlabel_Time = 'Time [s]';
m_to_cm = 1e2; %convert meters to centimeters;
rad_to_deg = 180 / pi; %convert radians to degrees;
N_to_uN = 1e6; %convert newtons to millnewtons

% model = 'full_feedback_system';
% varList = Simulink.findVars(model, 'WorkspaceType', 'base')
% Simulink.saveVars('ionocraft_drag_data.m','ModelParam')

sim('iros2018_AoA.slx');

state_vector.Data(abs(state_vector.Data) < 1e-5) = 0; %get rid of anything less than 10um, 10um/s, 10urad, 10urad/s
thrust_and_torque_inputs.Data(abs(thrust_and_torque_inputs.Data) < 1e-9) = 0; %get rid of anything less than 1nN or 100nN*cm


%% 3D PLOT - Attempt
%{
% figure;
% X = state_vector.Data(:,1);
% Y = state_vector.Data(:,2);
% Z = state_vector.Data(:,3);
% curve = animatedline('LineWidth',2);
% set(gca,'XLim',[0 50],'YLim',[0 50],'ZLim',[0 50]);
% view(43,24);
% hold on;
% grid on;
% xlabel('X [cm]');
% ylabel('Y [cm]');
% zlabel('Z [cm]');
% for i=1:50:length(X)
%     %addpoints(curve,X(i)*m_to_cm,Y(i)*m_to_cm,Z(i)*m_to_cm);
%     head_xyz = scatter3(X(i)*m_to_cm,Y(i)*m_to_cm,Z(i)*m_to_cm,'o','filled','MarkerFaceColor','b','MarkerEdgeColor','b');
%     pause(0.001);
%     head_xy = scatter3(X(i)*m_to_cm,Y(i)*m_to_cm,0,'s','filled','MarkerFaceColor','k','MarkerEdgeColor','k');
%     drawnow
%     %delete(head_xyz);
%     %delete(head_xy);
% end
%}
%% 3D QUIVER PLOT
if video_flag == 1          % Should change it so only displays to flag is on, and different flag for recording
    % TODO: Timings

    % Open Video Writer
    v = VideoWriter('3D_simulation.avi');
    open(v);

    figure;
%     subplot(1,2,1);
    X = state_vector.Data(:,1);
    Y = state_vector.Data(:,2);
    Z = state_vector.Data(:,3);
    psi = state_vector.Data(:,4);
    theta = state_vector.Data(:,5);
    phi = state_vector.Data(:,6);
    
    
    curve = animatedline('LineWidth',2);
    
    set(gca,'XLim',x_lim,'YLim',y_lim,'ZLim',z_lim);
    view(view_1,view_2);
    hold on;
    grid on;
    xlabel('X [cm]');
    ylabel('Y [cm]');
    zlabel('Z [cm]');

    %plot X on xy plane
    quiver3(-1000,-1000,0,5000,5000,0,'ShowArrowHead','Off','Color','k');
    quiver3(-1000,1000,0,5000,-5000,0,'ShowArrowHead','Off','Color','k');


    for i=1:video_frame_frequency:length(X)
        yaw = psi(i);
        pitch = theta(i);
        roll = phi(i);
        ionocraft_length = 1; %[cm], length of ionocraft in simulation

        %% Craft
        
        %compute ionocraft length vectors in the global frame
        x_vec_global_pos = vpa(subs(Body2Global)) * [ionocraft_length; 0; 0];
        x_vec_global_neg = vpa(subs(Body2Global)) * [-ionocraft_length; 0; 0];
        y_vec_global_pos = vpa(subs(Body2Global)) * [0; ionocraft_length; 0];
        y_vec_global_neg = vpa(subs(Body2Global)) * [0; -ionocraft_length; 0];

        %plot four main ionocraft length vectors in the global frame
        x_vec_global_pos_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,Z(i)*m_to_cm,x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','r');
        x_vec_global_neg_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,Z(i)*m_to_cm,x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
        y_vec_global_pos_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,Z(i)*m_to_cm,y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
        y_vec_global_neg_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,Z(i)*m_to_cm,y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

        %plot four offset x body frame vectors in the inertial frame
        x_vec_global_pos_offset_1_head = quiver3(X(i)*m_to_cm + y_vec_global_pos(1),Y(i)*m_to_cm + y_vec_global_pos(2),Z(i)*m_to_cm + y_vec_global_pos(3),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
        x_vec_global_pos_offset_2_head = quiver3(X(i)*m_to_cm + y_vec_global_neg(1),Y(i)*m_to_cm + y_vec_global_neg(2),Z(i)*m_to_cm + y_vec_global_neg(3),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
        x_vec_global_neg_offset_1_head = quiver3(X(i)*m_to_cm + y_vec_global_pos(1),Y(i)*m_to_cm + y_vec_global_pos(2),Z(i)*m_to_cm + y_vec_global_pos(3),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
        x_vec_global_neg_offset_2_head = quiver3(X(i)*m_to_cm + y_vec_global_neg(1),Y(i)*m_to_cm + y_vec_global_neg(2),Z(i)*m_to_cm + y_vec_global_neg(3),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

        %plot four offset y body frame vectors in the inertial frame
        y_vec_global_pos_offset_1_head = quiver3(X(i)*m_to_cm + x_vec_global_pos(1),Y(i)*m_to_cm + x_vec_global_pos(2),Z(i)*m_to_cm + x_vec_global_pos(3),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
        y_vec_global_pos_offset_2_head = quiver3(X(i)*m_to_cm + x_vec_global_neg(1),Y(i)*m_to_cm + x_vec_global_neg(2),Z(i)*m_to_cm + x_vec_global_neg(3),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
        y_vec_global_neg_offset_1_head = quiver3(X(i)*m_to_cm + x_vec_global_pos(1),Y(i)*m_to_cm + x_vec_global_pos(2),Z(i)*m_to_cm + x_vec_global_pos(3),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
        y_vec_global_neg_offset_2_head = quiver3(X(i)*m_to_cm + x_vec_global_neg(1),Y(i)*m_to_cm + x_vec_global_neg(2),Z(i)*m_to_cm + x_vec_global_neg(3),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

        
        %% Shadow
        
        %plot four main ionocraft length vectors projected onto the xy plane of the global frame
        x_vec_global_pos_xy_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,0,x_vec_global_pos(1),x_vec_global_pos(2),0,'ShowArrowHead','Off','Color','r');
        x_vec_global_neg_xy_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,0,x_vec_global_neg(1),x_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');
        y_vec_global_pos_xy_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,0,y_vec_global_pos(1),y_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
        y_vec_global_neg_xy_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,0,y_vec_global_neg(1),y_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');

        %plot four offset x body frame vectors in the xy plane of the global frame
        x_vec_global_pos_xy_offset_1_head = quiver3(X(i)*m_to_cm + y_vec_global_pos(1),Y(i)*m_to_cm + y_vec_global_pos(2),0,x_vec_global_pos(1),x_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
        x_vec_global_pos_xy_offset_2_head = quiver3(X(i)*m_to_cm + y_vec_global_neg(1),Y(i)*m_to_cm + y_vec_global_neg(2),0,x_vec_global_pos(1),x_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
        x_vec_global_neg_xy_offset_1_head = quiver3(X(i)*m_to_cm + y_vec_global_pos(1),Y(i)*m_to_cm + y_vec_global_pos(2),0,x_vec_global_neg(1),x_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');
        x_vec_global_neg_xy_offset_2_head = quiver3(X(i)*m_to_cm + y_vec_global_neg(1),Y(i)*m_to_cm + y_vec_global_neg(2),0,x_vec_global_neg(1),x_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');

        %plot four offset y body frame vectors in the xy plane of the global frame
        y_vec_global_pos_xy_offset_1_head = quiver3(X(i)*m_to_cm + x_vec_global_pos(1),Y(i)*m_to_cm + x_vec_global_pos(2),0,y_vec_global_pos(1),y_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
        y_vec_global_pos_xy_offset_2_head = quiver3(X(i)*m_to_cm + x_vec_global_neg(1),Y(i)*m_to_cm + x_vec_global_neg(2),0,y_vec_global_pos(1),y_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
        y_vec_global_neg_xy_offset_1_head = quiver3(X(i)*m_to_cm + x_vec_global_pos(1),Y(i)*m_to_cm + x_vec_global_pos(2),0,y_vec_global_neg(1),y_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');
        y_vec_global_neg_xy_offset_2_head = quiver3(X(i)*m_to_cm + x_vec_global_neg(1),Y(i)*m_to_cm + x_vec_global_neg(2),0,y_vec_global_neg(1),y_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');

        %% Insert zoomed here
        
        
        
        
        %% Actual plotting
        drawnow

        frame = getframe(gcf);
        writeVideo(v,frame);
        
        %% Deleting for next frame
        
        delete(x_vec_global_pos_head);
        delete(x_vec_global_neg_head);
        delete(y_vec_global_pos_head);
        delete(y_vec_global_neg_head);

        delete(x_vec_global_pos_offset_1_head);
        delete(x_vec_global_pos_offset_2_head);
        delete(x_vec_global_neg_offset_1_head);
        delete(x_vec_global_neg_offset_2_head);

        delete(y_vec_global_pos_offset_1_head);
        delete(y_vec_global_pos_offset_2_head);
        delete(y_vec_global_neg_offset_1_head);
        delete(y_vec_global_neg_offset_2_head);

        delete(x_vec_global_pos_xy_head);
        delete(x_vec_global_neg_xy_head);
        delete(y_vec_global_pos_xy_head);
        delete(y_vec_global_neg_xy_head);

        delete(x_vec_global_pos_xy_offset_1_head);
        delete(x_vec_global_pos_xy_offset_2_head);
        delete(x_vec_global_neg_xy_offset_1_head);
        delete(x_vec_global_neg_xy_offset_2_head);

        delete(y_vec_global_pos_xy_offset_1_head);
        delete(y_vec_global_pos_xy_offset_2_head);
        delete(y_vec_global_neg_xy_offset_1_head);
        delete(y_vec_global_neg_xy_offset_2_head);


    end

    %% Last Frame
    
    %plot four main ionocraft length vectors in the global frame
    x_vec_global_pos_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,Z(i)*m_to_cm,x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','r');
    x_vec_global_neg_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,Z(i)*m_to_cm,x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_pos_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,Z(i)*m_to_cm,y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_neg_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,Z(i)*m_to_cm,y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

    %plot four offset x body framvectors in the inertial frame
    x_vec_global_pos_offset_1_head = quiver3(X(i)*m_to_cm + y_vec_global_pos(1),Y(i)*m_to_cm + y_vec_global_pos(2),Z(i)*m_to_cm + y_vec_global_pos(3),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    x_vec_global_pos_offset_2_head = quiver3(X(i)*m_to_cm + y_vec_global_neg(1),Y(i)*m_to_cm + y_vec_global_neg(2),Z(i)*m_to_cm + y_vec_global_neg(3),x_vec_global_pos(1),x_vec_global_pos(2),x_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    x_vec_global_neg_offset_1_head = quiver3(X(i)*m_to_cm + y_vec_global_pos(1),Y(i)*m_to_cm + y_vec_global_pos(2),Z(i)*m_to_cm + y_vec_global_pos(3),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
    x_vec_global_neg_offset_2_head = quiver3(X(i)*m_to_cm + y_vec_global_neg(1),Y(i)*m_to_cm + y_vec_global_neg(2),Z(i)*m_to_cm + y_vec_global_neg(3),x_vec_global_neg(1),x_vec_global_neg(2),x_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

    %plot four offset y body frame vectors in the inertial frame
    y_vec_global_pos_offset_1_head = quiver3(X(i)*m_to_cm + x_vec_global_pos(1),Y(i)*m_to_cm + x_vec_global_pos(2),Z(i)*m_to_cm + x_vec_global_pos(3),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_pos_offset_2_head = quiver3(X(i)*m_to_cm + x_vec_global_neg(1),Y(i)*m_to_cm + x_vec_global_neg(2),Z(i)*m_to_cm + x_vec_global_neg(3),y_vec_global_pos(1),y_vec_global_pos(2),y_vec_global_pos(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_neg_offset_1_head = quiver3(X(i)*m_to_cm + x_vec_global_pos(1),Y(i)*m_to_cm + x_vec_global_pos(2),Z(i)*m_to_cm + x_vec_global_pos(3),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');
    y_vec_global_neg_offset_2_head = quiver3(X(i)*m_to_cm + x_vec_global_neg(1),Y(i)*m_to_cm + x_vec_global_neg(2),Z(i)*m_to_cm + x_vec_global_neg(3),y_vec_global_neg(1),y_vec_global_neg(2),y_vec_global_neg(3),'ShowArrowHead','Off','Color','b');

    %plot four main ionocraft length vectors projected onto the xy plane of the global frame
    x_vec_global_pos_yaw_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,0,x_vec_global_pos(1),x_vec_global_pos(2),0,'ShowArrowHead','Off','Color','r');
    x_vec_global_neg_yaw_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,0,x_vec_global_neg(1),x_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');
    y_vec_global_pos_yaw_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,0,y_vec_global_pos(1),y_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
    y_vec_global_neg_yaw_head = quiver3(X(i)*m_to_cm,Y(i)*m_to_cm,0,y_vec_global_neg(1),y_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');

    %plot four offset x body frame vectors in the xy plane of the global frame
    x_vec_global_pos_xy_offset_1_head = quiver3(X(i)*m_to_cm + y_vec_global_pos(1),Y(i)*m_to_cm + y_vec_global_pos(2),0,x_vec_global_pos(1),x_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
    x_vec_global_pos_xy_offset_2_head = quiver3(X(i)*m_to_cm + y_vec_global_neg(1),Y(i)*m_to_cm + y_vec_global_neg(2),0,x_vec_global_pos(1),x_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
    x_vec_global_neg_xy_offset_1_head = quiver3(X(i)*m_to_cm + y_vec_global_pos(1),Y(i)*m_to_cm + y_vec_global_pos(2),0,x_vec_global_neg(1),x_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');
    x_vec_global_neg_xy_offset_2_head = quiver3(X(i)*m_to_cm + y_vec_global_neg(1),Y(i)*m_to_cm + y_vec_global_neg(2),0,x_vec_global_neg(1),x_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');

    %plot four offset y body frame vectors in the xy plane of the global frame
    y_vec_global_pos_xy_offset_1_head = quiver3(X(i)*m_to_cm + x_vec_global_pos(1),Y(i)*m_to_cm + x_vec_global_pos(2),0,y_vec_global_pos(1),y_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
    y_vec_global_pos_xy_offset_2_head = quiver3(X(i)*m_to_cm + x_vec_global_neg(1),Y(i)*m_to_cm + x_vec_global_neg(2),0,y_vec_global_pos(1),y_vec_global_pos(2),0,'ShowArrowHead','Off','Color','k');
    y_vec_global_neg_xy_offset_1_head = quiver3(X(i)*m_to_cm + x_vec_global_pos(1),Y(i)*m_to_cm + x_vec_global_pos(2),0,y_vec_global_neg(1),y_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');
    y_vec_global_neg_xy_offset_2_head = quiver3(X(i)*m_to_cm + x_vec_global_neg(1),Y(i)*m_to_cm + x_vec_global_neg(2),0,y_vec_global_neg(1),y_vec_global_neg(2),0,'ShowArrowHead','Off','Color','k');

    %% Close Video Writer
    close(v);

end
%% STATE VARIABLES PLOT
figure;
for i = 1:12
    subplot(state_vector_rows,state_vector_cols,i);
    if i == 1
        plot(state_vector.Time,state_vector.Data(:,i) * m_to_cm);
        ylabel('X [cm]');
        title('X');
    elseif i == 2
        plot(state_vector.Time,state_vector.Data(:,i) * m_to_cm);
        ylabel('Y [cm]');
        title('Y');
    elseif i == 3
        plot(state_vector.Time,state_vector.Data(:,i) * m_to_cm);
        ylabel('Z [cm]');
        title('Z');
    elseif i == 12
        hold on
        plot(state_vector.Time,estimates.Data(:,3), 'color',[0.9100    0.4100    0.1700]);
         plot(state_vector.Time,accel_z.Data(:,1), 'b');
        ylabel('Z Accel [m/s^2]');
        title('Z Accel');
        hold off
    elseif i == 4
        plot(state_vector.Time,state_vector.Data(:,i) * rad_to_deg);
        ylabel('yaw [deg]');
        title('Yaw');
    elseif i == 5
        hold on
        plot(state_vector.Time,estimates.Data(:,i) * rad_to_deg, 'color',[0.9100    0.4100    0.1700]);
        plot(state_vector.Time,state_vector.Data(:,i) * rad_to_deg, 'b');
        ylabel('pitch [deg]');
        title('Pitch');
        hold off
    elseif i == 6
        hold on
        plot(state_vector.Time,estimates.Data(:,i) * rad_to_deg, 'color',[0.9100    0.4100    0.1700]);
        plot(state_vector.Time,state_vector.Data(:,i) * rad_to_deg, 'b');
        ylabel('roll [deg]');
        title('Roll');
        hold off
    elseif i == 7
        plot(state_vector.Time,state_vector.Data(:,i) * m_to_cm);
        ylabel('vx [cm/s]');
        title('vx');
    elseif i == 8
        plot(state_vector.Time,state_vector.Data(:,i) * m_to_cm);
        ylabel('vy [cm/s]');
        title('vy');
    elseif i == 9
        plot(state_vector.Time,state_vector.Data(:,i) * m_to_cm);
        ylabel('vz [cm/s]');
        title('vz');
    elseif i == 10
        plot(state_vector.Time,state_vector.Data(:,i) * rad_to_deg);
        ylabel('wx [deg/s]');
        title('wx');
    elseif i == 11
        plot(state_vector.Time,state_vector.Data(:,i) * rad_to_deg);
        ylabel('wy [deg/s]');
        title('wy');
    elseif i == 120
        plot(state_vector.Time,state_vector.Data(:,i) * rad_to_deg);
        ylabel('wz [deg/s]');
        title('wz');
    end
    xlabel(xlabel_Time);
end

%% THRUST AND TORQUE INPUT PLOTS
%{
% thrust_and_torque_inputs_rows = 4;
% thrust_and_torque_inputs_cols = 1;
% x_label_Time = 'Time [s]';
% figure;
% for i = 1:4
%     subplot(thrust_and_torque_inputs_rows, thrust_and_torque_inputs_cols, i);
%     if i == 1
%         plot(thrust_and_torque_inputs.Time,thrust_and_torque_inputs.Data(:,i) * N_to_uN);
%         ylabel('T [uN]');
%         title('Thrust');
%     elseif i == 2
%         plot(thrust_and_torque_inputs.Time,thrust_and_torque_inputs.Data(:,i) * N_to_uN * m_to_cm);
%         ylabel('tauz [uN x cm]');
%         title('Tau Z');
%     elseif i == 3
%         plot(thrust_and_torque_inputs.Time,thrust_and_torque_inputs.Data(:,i) * N_to_uN * m_to_cm);
%         ylabel('tauy [uN x cm]');
%         title('Tau Y');
%     elseif i == 4
%         plot(thrust_and_torque_inputs.Time,thrust_and_torque_inputs.Data(:,i) * N_to_uN * m_to_cm);
%         ylabel('taux [uN x cm]');
%         title('Tau X');
%     end
%     xlabel(xlabel_Time);
% end
%}
%% FORCE INPUT PLOTS

force_inputs_rows = 4;
force_inputs_cols = 1;
x_label_Time = 'Time [s]';
figure;
for i = 1:4
    subplot(force_inputs_rows, force_inputs_cols, i);
    if i == 1
        plot(force_inputs.Time,force_inputs.Data(:,i) * N_to_uN);
        ylabel('F4 [uN]');
        title('F4');
    elseif i == 2
        plot(force_inputs.Time,force_inputs.Data(:,i) * N_to_uN);
        ylabel('F3 [uN]');
        title('F3');
    elseif i == 3
        plot(force_inputs.Time,force_inputs.Data(:,i) * N_to_uN);
        ylabel('F2 [uN]');
        title('F2');
    elseif i == 4
        plot(force_inputs.Time,force_inputs.Data(:,i) * N_to_uN);
        ylabel('F1 [uN]');
        title('F1');
    end
    xlabel(xlabel_Time);
end



