%% ========================================================================
%  SIMULATION OF 3D ROBOTIC ARM
%  ========================================================================
%  Description : 3-DOF Robotic Arm Simulation with Forward & Inverse
%                Kinematics using Denavit-Hartenberg (DH) Parameters
%  Author      : Sandeep Gupta
%  Platform    : MATLAB R2020a or later
%  ========================================================================

clc;
clear all;
close all;

%% ======================== ROBOT PARAMETERS ==============================
% Link lengths (in meters)
L1 = 1.0;   % Length of Link 1 (Base to Shoulder)
L2 = 0.8;   % Length of Link 2 (Shoulder to Elbow)
L3 = 0.6;   % Length of Link 3 (Elbow to End-Effector)

% Joint angle limits (in degrees)
theta1_limits = [-180, 180];  % Base rotation
theta2_limits = [-90, 90];    % Shoulder rotation
theta3_limits = [-135, 135];  % Elbow rotation

%% ===================== DH PARAMETERS TABLE ==============================
% DH Parameters: [theta, d, a, alpha]
% theta : Joint angle (variable for revolute joints)
% d     : Link offset along z-axis
% a     : Link length along x-axis
% alpha : Twist angle about x-axis

fprintf('╔══════════════════════════════════════════════════════╗\n');
fprintf('║     SIMULATION OF 3D ROBOTIC ARM (3-DOF)           ║\n');
fprintf('║     Using Denavit-Hartenberg Convention             ║\n');
fprintf('╚══════════════════════════════════════════════════════╝\n\n');

%% =================== FORWARD KINEMATICS FUNCTION ========================
fprintf('>> Initializing Forward Kinematics Engine...\n');

% Define initial joint angles (in degrees)
theta1 = 45;   % Base rotation
theta2 = 30;   % Shoulder angle
theta3 = -45;  % Elbow angle

% Convert to radians
t1 = deg2rad(theta1);
t2 = deg2rad(theta2);
t3 = deg2rad(theta3);

% DH Transformation Matrices
T01 = dh_transform(t1, L1, 0, pi/2);    % Base to Link 1
T12 = dh_transform(t2, 0, L2, 0);       % Link 1 to Link 2
T23 = dh_transform(t3, 0, L3, 0);       % Link 2 to Link 3

% Complete transformation from base to end-effector
T02 = T01 * T12;
T03 = T01 * T12 * T23;

% Extract joint positions for plotting
P0 = [0; 0; 0];                          % Base origin
P1 = T01(1:3, 4);                        % Joint 1 position
P2 = T02(1:3, 4);                        % Joint 2 position
P3 = T03(1:3, 4);                        % End-effector position

% Display end-effector position
fprintf('\n>> End-Effector Position:\n');
fprintf('   X = %.4f m\n', P3(1));
fprintf('   Y = %.4f m\n', P3(2));
fprintf('   Z = %.4f m\n', P3(3));

%% ====================== 3D VISUALIZATION ================================
fprintf('\n>> Rendering 3D Robotic Arm...\n');

figure('Name', 'Simulation of 3D Robotic Arm', ...
       'NumberTitle', 'off', ...
       'Color', [0.1 0.1 0.15], ...
       'Position', [100 100 1200 800]);

% ---- Plot Robot Arm ----
plot_robot_arm(P0, P1, P2, P3, L1, L2, L3);

%% ================== WORKSPACE ANALYSIS ==================================
fprintf('\n>> Computing Workspace Envelope...\n');

figure('Name', 'Workspace Analysis', ...
       'NumberTitle', 'off', ...
       'Color', [0.1 0.1 0.15], ...
       'Position', [150 150 1200 800]);

% Generate workspace points
N = 20; % Resolution
theta1_range = linspace(deg2rad(theta1_limits(1)), deg2rad(theta1_limits(2)), N);
theta2_range = linspace(deg2rad(theta2_limits(1)), deg2rad(theta2_limits(2)), N);
theta3_range = linspace(deg2rad(theta3_limits(1)), deg2rad(theta3_limits(2)), N);

workspace_points = [];

for i = 1:length(theta1_range)
    for j = 1:length(theta2_range)
        for k = 1:length(theta3_range)
            T_temp = dh_transform(theta1_range(i), L1, 0, pi/2) * ...
                     dh_transform(theta2_range(j), 0, L2, 0) * ...
                     dh_transform(theta3_range(k), 0, L3, 0);
            workspace_points = [workspace_points; T_temp(1:3, 4)'];
        end
    end
end

% Plot workspace envelope
scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), ...
         2, workspace_points(:,3), 'filled', 'MarkerFaceAlpha', 0.3);
colormap(jet);
colorbar('Color', 'white');

hold on;
% Overlay the current arm configuration
plot_robot_arm_simple(P0, P1, P2, P3);
hold off;

title('Robotic Arm Workspace Envelope', 'Color', 'white', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('X (m)', 'Color', 'white', 'FontSize', 12);
ylabel('Y (m)', 'Color', 'white', 'FontSize', 12);
zlabel('Z (m)', 'Color', 'white', 'FontSize', 12);
set(gca, 'Color', [0.15 0.15 0.2], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
axis equal;
grid on;
set(gca, 'GridColor', [0.3 0.3 0.4], 'GridAlpha', 0.5);
view(45, 30);

fprintf('   Total workspace points computed: %d\n', size(workspace_points, 1));

%% ================ JOINT TRAJECTORY ANIMATION ============================
fprintf('\n>> Generating Joint Trajectory Animation...\n');

figure('Name', 'Trajectory Animation', ...
       'NumberTitle', 'off', ...
       'Color', [0.1 0.1 0.15], ...
       'Position', [200 50 1200 800]);

% Define trajectory: smooth sinusoidal motion
t_sim = linspace(0, 2*pi, 100);
theta1_traj = 60 * sin(t_sim);                   % Base oscillation
theta2_traj = 30 * sin(2*t_sim) + 15;            % Shoulder oscillation
theta3_traj = 45 * cos(t_sim) - 20;              % Elbow oscillation

% Pre-compute trajectory points for end-effector trace
ee_trace = zeros(length(t_sim), 3);

for frame = 1:length(t_sim)
    cla;
    
    t1_f = deg2rad(theta1_traj(frame));
    t2_f = deg2rad(theta2_traj(frame));
    t3_f = deg2rad(theta3_traj(frame));
    
    T01_f = dh_transform(t1_f, L1, 0, pi/2);
    T02_f = T01_f * dh_transform(t2_f, 0, L2, 0);
    T03_f = T02_f * dh_transform(t3_f, 0, L3, 0);
    
    P0_f = [0; 0; 0];
    P1_f = T01_f(1:3, 4);
    P2_f = T02_f(1:3, 4);
    P3_f = T03_f(1:3, 4);
    
    ee_trace(frame, :) = P3_f';
    
    % Plot arm
    plot_robot_arm(P0_f, P1_f, P2_f, P3_f, L1, L2, L3);
    
    % Plot end-effector trace
    hold on;
    if frame > 1
        plot3(ee_trace(1:frame, 1), ee_trace(1:frame, 2), ee_trace(1:frame, 3), ...
              'Color', [1 0.4 0.2 0.7], 'LineWidth', 2);
    end
    hold off;
    
    title(sprintf('Trajectory Animation — Frame %d/%d', frame, length(t_sim)), ...
          'Color', 'white', 'FontSize', 14, 'FontWeight', 'bold');
    
    drawnow;
    pause(0.03);
end

fprintf('   Animation complete! (%d frames rendered)\n', length(t_sim));

%% ================ INVERSE KINEMATICS ====================================
fprintf('\n>> Computing Inverse Kinematics Solution...\n');

% Target end-effector position
target_pos = [0.8; 0.5; 1.2];
fprintf('   Target Position: [%.2f, %.2f, %.2f] m\n', target_pos(1), target_pos(2), target_pos(3));

[ik_theta1, ik_theta2, ik_theta3, ik_success] = inverse_kinematics(target_pos, L1, L2, L3);

if ik_success
    fprintf('   ✓ IK Solution Found!\n');
    fprintf('   θ1 = %.2f°, θ2 = %.2f°, θ3 = %.2f°\n', ...
            rad2deg(ik_theta1), rad2deg(ik_theta2), rad2deg(ik_theta3));
    
    % Verify with forward kinematics
    T_verify = dh_transform(ik_theta1, L1, 0, pi/2) * ...
               dh_transform(ik_theta2, 0, L2, 0) * ...
               dh_transform(ik_theta3, 0, L3, 0);
    fk_pos = T_verify(1:3, 4);
    error_dist = norm(fk_pos - target_pos);
    fprintf('   FK Verification Error: %.6f m\n', error_dist);
else
    fprintf('   ✗ Target position is outside the workspace!\n');
end

%% =================== JOINT ANGLE PLOTS ==================================
fprintf('\n>> Generating Joint Angle Analysis Plots...\n');

figure('Name', 'Joint Angle Analysis', ...
       'NumberTitle', 'off', ...
       'Color', [0.1 0.1 0.15], ...
       'Position', [250 100 1200 600]);

time_vec = linspace(0, 10, length(t_sim));

subplot(3, 1, 1);
plot(time_vec, theta1_traj, 'Color', [0.2 0.6 1], 'LineWidth', 2);
title('Joint 1 — Base Rotation', 'Color', 'white', 'FontSize', 12);
ylabel('θ₁ (deg)', 'Color', 'white', 'FontSize', 11);
set(gca, 'Color', [0.15 0.15 0.2], 'XColor', 'w', 'YColor', 'w');
grid on; set(gca, 'GridColor', [0.3 0.3 0.4]);

subplot(3, 1, 2);
plot(time_vec, theta2_traj, 'Color', [0.2 0.9 0.4], 'LineWidth', 2);
title('Joint 2 — Shoulder Rotation', 'Color', 'white', 'FontSize', 12);
ylabel('θ₂ (deg)', 'Color', 'white', 'FontSize', 11);
set(gca, 'Color', [0.15 0.15 0.2], 'XColor', 'w', 'YColor', 'w');
grid on; set(gca, 'GridColor', [0.3 0.3 0.4]);

subplot(3, 1, 3);
plot(time_vec, theta3_traj, 'Color', [1 0.4 0.2], 'LineWidth', 2);
title('Joint 3 — Elbow Rotation', 'Color', 'white', 'FontSize', 12);
ylabel('θ₃ (deg)', 'Color', 'white', 'FontSize', 11);
xlabel('Time (s)', 'Color', 'white', 'FontSize', 11);
set(gca, 'Color', [0.15 0.15 0.2], 'XColor', 'w', 'YColor', 'w');
grid on; set(gca, 'GridColor', [0.3 0.3 0.4]);

sgtitle('Joint Angle Trajectory Analysis', 'Color', 'white', 'FontSize', 16, 'FontWeight', 'bold');

fprintf('\n>> Simulation Complete!\n');
fprintf('══════════════════════════════════════════════════════\n');

%% ======================== HELPER FUNCTIONS ==============================

function T = dh_transform(theta, d, a, alpha)
    % Compute the Denavit-Hartenberg Transformation Matrix
    %
    % Parameters:
    %   theta - Joint angle (radians)
    %   d     - Link offset
    %   a     - Link length
    %   alpha - Link twist (radians)
    %
    % Returns:
    %   T     - 4x4 Homogeneous Transformation Matrix
    
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

function plot_robot_arm(P0, P1, P2, P3, L1, L2, L3)
    % Plot the 3D robotic arm with styled joints and links
    
    % Joint positions matrix
    joints = [P0'; P1'; P2'; P3'];
    
    % Plot links
    plot3(joints(:,1), joints(:,2), joints(:,3), ...
          'Color', [0.3 0.7 1], 'LineWidth', 4);
    hold on;
    
    % Plot link outlines (thicker background)
    plot3(joints(:,1), joints(:,2), joints(:,3), ...
          'Color', [0.1 0.3 0.6], 'LineWidth', 8);
    
    % Re-plot links on top
    plot3(joints(:,1), joints(:,2), joints(:,3), ...
          'Color', [0.3 0.7 1], 'LineWidth', 4);
    
    % Plot joints
    joint_colors = [0.2 0.8 0.3;    % Base - Green
                    1.0 0.8 0.0;     % Shoulder - Yellow
                    1.0 0.4 0.1;     % Elbow - Orange
                    1.0 0.2 0.3];    % End-Effector - Red
    
    joint_sizes = [120, 100, 80, 60];
    joint_labels = {'Base', 'Shoulder', 'Elbow', 'End-Effector'};
    
    for i = 1:4
        scatter3(joints(i,1), joints(i,2), joints(i,3), ...
                 joint_sizes(i), joint_colors(i,:), 'filled', ...
                 'MarkerEdgeColor', 'white', 'LineWidth', 1.5);
        text(joints(i,1)+0.05, joints(i,2)+0.05, joints(i,3)+0.1, ...
             joint_labels{i}, 'Color', joint_colors(i,:), ...
             'FontSize', 10, 'FontWeight', 'bold');
    end
    
    % Plot base platform
    theta_circle = linspace(0, 2*pi, 50);
    r_base = 0.15;
    x_base = r_base * cos(theta_circle);
    y_base = r_base * sin(theta_circle);
    z_base = zeros(size(theta_circle));
    fill3(x_base, y_base, z_base, [0.3 0.3 0.4], ...
          'FaceAlpha', 0.6, 'EdgeColor', [0.5 0.5 0.6]);
    
    % Plot coordinate frames at each joint
    frame_scale = 0.15;
    % Base frame
    quiver3(0, 0, 0, frame_scale, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    quiver3(0, 0, 0, 0, frame_scale, 0, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    quiver3(0, 0, 0, 0, 0, frame_scale, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    
    hold off;
    
    % Styling
    title(sprintf('3D Robotic Arm Configuration\nEnd-Effector: [%.3f, %.3f, %.3f] m', ...
          P3(1), P3(2), P3(3)), ...
          'Color', 'white', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X (m)', 'Color', 'white', 'FontSize', 12);
    ylabel('Y (m)', 'Color', 'white', 'FontSize', 12);
    zlabel('Z (m)', 'Color', 'white', 'FontSize', 12);
    
    set(gca, 'Color', [0.15 0.15 0.2], ...
             'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    
    axis equal;
    grid on;
    set(gca, 'GridColor', [0.3 0.3 0.4], 'GridAlpha', 0.5);
    
    max_reach = L1 + L2 + L3;
    xlim([-max_reach, max_reach]);
    ylim([-max_reach, max_reach]);
    zlim([-0.2, max_reach + 0.5]);
    
    view(45, 25);
    
    % Add legend
    legend({'Links', '', '', 'Base', 'Shoulder', 'Elbow', 'End-Effector'}, ...
           'TextColor', 'white', 'Color', [0.2 0.2 0.25], ...
           'EdgeColor', [0.4 0.4 0.5], 'Location', 'northeast');
end

function plot_robot_arm_simple(P0, P1, P2, P3)
    % Simplified arm plot for overlay on workspace
    joints = [P0'; P1'; P2'; P3'];
    plot3(joints(:,1), joints(:,2), joints(:,3), ...
          'w-', 'LineWidth', 3);
    scatter3(joints(:,1), joints(:,2), joints(:,3), ...
             80, 'w', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
end

function [theta1, theta2, theta3, success] = inverse_kinematics(target, L1, L2, L3)
    % Compute Inverse Kinematics for 3-DOF robotic arm
    % using geometric approach
    %
    % Parameters:
    %   target - [x; y; z] desired end-effector position
    %   L1, L2, L3 - Link lengths
    %
    % Returns:
    %   theta1, theta2, theta3 - Joint angles (radians)
    %   success - Boolean indicating if solution exists
    
    x = target(1);
    y = target(2);
    z = target(3);
    
    success = true;
    
    % Joint 1: Base rotation (rotation about z-axis)
    theta1 = atan2(y, x);
    
    % Project onto the plane containing links 2 and 3
    r = sqrt(x^2 + y^2);          % Horizontal distance
    s = z - L1;                    % Vertical distance from joint 1
    D = sqrt(r^2 + s^2);          % Distance from joint 1 to target
    
    % Check reachability
    if D > (L2 + L3) || D < abs(L2 - L3)
        theta1 = 0;
        theta2 = 0;
        theta3 = 0;
        success = false;
        return;
    end
    
    % Joint 3: Elbow angle (using cosine rule)
    cos_theta3 = (D^2 - L2^2 - L3^2) / (2 * L2 * L3);
    cos_theta3 = max(-1, min(1, cos_theta3)); % Clamp for numerical stability
    theta3 = acos(cos_theta3);     % Elbow-up solution
    
    % Joint 2: Shoulder angle
    beta = atan2(s, r);
    psi = atan2(L3 * sin(theta3), L2 + L3 * cos(theta3));
    theta2 = beta - psi;
    
    % Negate theta3 for elbow-up convention
    theta3 = -theta3;
end
