%% ========================================================================
%  TRAJECTORY PLANNING - 3D ROBOTIC ARM
%  ========================================================================
%  Description : Generates smooth trajectories using cubic polynomial
%                and quintic polynomial interpolation for the robotic arm
%  Author      : Sandeep Gupta
%  Platform    : MATLAB R2020a or later
%  ========================================================================

clc;
clear all;
close all;

%% Robot Parameters
L1 = 1.0;
L2 = 0.8;
L3 = 0.6;

fprintf('╔══════════════════════════════════════════════════════╗\n');
fprintf('║       TRAJECTORY PLANNING MODULE                    ║\n');
fprintf('╚══════════════════════════════════════════════════════╝\n\n');

%% Define Waypoints (Joint Space)
% Start configuration
q_start = [0; 0; 0];          % [theta1, theta2, theta3] degrees

% Via-points
q_via1 = [30; 20; -15];
q_via2 = [60; 40; -30];
q_via3 = [90; 20; -60];

% End configuration
q_end = [120; -10; -45];

waypoints = [q_start, q_via1, q_via2, q_via3, q_end];
n_segments = size(waypoints, 2) - 1;

fprintf('>> Waypoints defined: %d points, %d segments\n', size(waypoints,2), n_segments);

%% Cubic Polynomial Trajectory
fprintf('>> Computing Cubic Polynomial Trajectory...\n');

t_total = 10;  % Total time (seconds)
dt = 0.05;     % Time step
t_segment = t_total / n_segments;

t_full = [];
q_cubic = [];
qd_cubic = [];
qdd_cubic = [];

for seg = 1:n_segments
    t_seg = 0:dt:t_segment;
    q0 = waypoints(:, seg);
    qf = waypoints(:, seg+1);
    
    for joint = 1:3
        [pos, vel, acc] = cubic_trajectory(q0(joint), qf(joint), t_segment, t_seg);
        q_cubic_seg(joint, :) = pos;
        qd_cubic_seg(joint, :) = vel;
        qdd_cubic_seg(joint, :) = acc;
    end
    
    t_offset = (seg-1) * t_segment;
    t_full = [t_full, t_seg + t_offset];
    q_cubic = [q_cubic, q_cubic_seg];
    qd_cubic = [qd_cubic, qd_cubic_seg];
    qdd_cubic = [qdd_cubic, qdd_cubic_seg];
end

%% Quintic Polynomial Trajectory
fprintf('>> Computing Quintic Polynomial Trajectory...\n');

q_quintic = [];
qd_quintic = [];
qdd_quintic = [];
t_full_q = [];

for seg = 1:n_segments
    t_seg = 0:dt:t_segment;
    q0 = waypoints(:, seg);
    qf = waypoints(:, seg+1);
    
    for joint = 1:3
        [pos, vel, acc] = quintic_trajectory(q0(joint), qf(joint), t_segment, t_seg);
        q_quintic_seg(joint, :) = pos;
        qd_quintic_seg(joint, :) = vel;
        qdd_quintic_seg(joint, :) = acc;
    end
    
    t_offset = (seg-1) * t_segment;
    t_full_q = [t_full_q, t_seg + t_offset];
    q_quintic = [q_quintic, q_quintic_seg];
    qd_quintic = [qd_quintic, qd_quintic_seg];
    qdd_quintic = [qdd_quintic, qdd_quintic_seg];
end

%% Plot Trajectory Comparison
fprintf('>> Generating Trajectory Plots...\n');

figure('Name', 'Trajectory Planning Results', ...
       'NumberTitle', 'off', ...
       'Color', [0.1 0.1 0.15], ...
       'Position', [50 50 1400 900]);

joint_names = {'θ₁ (Base)', 'θ₂ (Shoulder)', 'θ₃ (Elbow)'};
joint_colors = {[0.2 0.6 1], [0.2 0.9 0.4], [1 0.4 0.2]};

for j = 1:3
    % Position
    subplot(3, 3, (j-1)*3 + 1);
    plot(t_full, q_cubic(j,:), 'Color', joint_colors{j}, 'LineWidth', 2);
    hold on;
    plot(t_full_q, q_quintic(j,:), '--', 'Color', joint_colors{j}*0.7+0.3, 'LineWidth', 2);
    scatter([0:t_segment:t_total], waypoints(j,:), 60, 'w', 'filled', 'MarkerEdgeColor', 'k');
    hold off;
    title([joint_names{j}, ' — Position'], 'Color', 'w', 'FontSize', 11);
    ylabel('Angle (°)', 'Color', 'w');
    if j == 3, xlabel('Time (s)', 'Color', 'w'); end
    legend({'Cubic', 'Quintic', 'Waypoints'}, 'TextColor', 'w', ...
           'Color', [0.2 0.2 0.25], 'EdgeColor', [0.4 0.4 0.5]);
    set(gca, 'Color', [0.15 0.15 0.2], 'XColor', 'w', 'YColor', 'w');
    grid on; set(gca, 'GridColor', [0.3 0.3 0.4]);
    
    % Velocity
    subplot(3, 3, (j-1)*3 + 2);
    plot(t_full, qd_cubic(j,:), 'Color', joint_colors{j}, 'LineWidth', 2);
    hold on;
    plot(t_full_q, qd_quintic(j,:), '--', 'Color', joint_colors{j}*0.7+0.3, 'LineWidth', 2);
    hold off;
    title([joint_names{j}, ' — Velocity'], 'Color', 'w', 'FontSize', 11);
    ylabel('°/s', 'Color', 'w');
    if j == 3, xlabel('Time (s)', 'Color', 'w'); end
    set(gca, 'Color', [0.15 0.15 0.2], 'XColor', 'w', 'YColor', 'w');
    grid on; set(gca, 'GridColor', [0.3 0.3 0.4]);
    
    % Acceleration
    subplot(3, 3, (j-1)*3 + 3);
    plot(t_full, qdd_cubic(j,:), 'Color', joint_colors{j}, 'LineWidth', 2);
    hold on;
    plot(t_full_q, qdd_quintic(j,:), '--', 'Color', joint_colors{j}*0.7+0.3, 'LineWidth', 2);
    hold off;
    title([joint_names{j}, ' — Acceleration'], 'Color', 'w', 'FontSize', 11);
    ylabel('°/s²', 'Color', 'w');
    if j == 3, xlabel('Time (s)', 'Color', 'w'); end
    set(gca, 'Color', [0.15 0.15 0.2], 'XColor', 'w', 'YColor', 'w');
    grid on; set(gca, 'GridColor', [0.3 0.3 0.4]);
end

sgtitle('Trajectory Planning: Cubic vs Quintic Polynomial', ...
        'Color', 'white', 'FontSize', 16, 'FontWeight', 'bold');

%% Animate Trajectory on Robot
fprintf('>> Animating Trajectory on Robot...\n');

figure('Name', 'Trajectory Execution', ...
       'NumberTitle', 'off', ...
       'Color', [0.1 0.1 0.15], ...
       'Position', [100 100 1000 800]);

ee_path = zeros(length(t_full_q), 3);

for frame = 1:3:length(t_full_q)  % Skip frames for speed
    cla;
    
    t1 = deg2rad(q_quintic(1, frame));
    t2 = deg2rad(q_quintic(2, frame));
    t3 = deg2rad(q_quintic(3, frame));
    
    T01 = dh_transform(t1, L1, 0, pi/2);
    T02 = T01 * dh_transform(t2, 0, L2, 0);
    T03 = T02 * dh_transform(t3, 0, L3, 0);
    
    P0 = [0;0;0]; P1 = T01(1:3,4); P2 = T02(1:3,4); P3 = T03(1:3,4);
    ee_path(frame, :) = P3';
    
    % Plot arm
    joints = [P0'; P1'; P2'; P3'];
    plot3(joints(:,1), joints(:,2), joints(:,3), ...
          'Color', [0.1 0.3 0.6], 'LineWidth', 6);
    hold on;
    plot3(joints(:,1), joints(:,2), joints(:,3), ...
          'Color', [0.3 0.7 1], 'LineWidth', 3);
    
    colors = [0.2 0.8 0.3; 1 0.8 0; 1 0.4 0.1; 1 0.2 0.3];
    for i = 1:4
        scatter3(joints(i,1), joints(i,2), joints(i,3), ...
                 [100 80 70 50], colors(i,:), 'filled', ...
                 'MarkerEdgeColor', 'w');
    end
    
    % Plot path trace
    valid_idx = find(any(ee_path, 2));
    if ~isempty(valid_idx)
        plot3(ee_path(valid_idx, 1), ee_path(valid_idx, 2), ee_path(valid_idx, 3), ...
              'Color', [1 0.4 0.2 0.7], 'LineWidth', 2);
    end
    
    hold off;
    
    max_reach = L1 + L2 + L3;
    title(sprintf('Trajectory Execution — t = %.2f s', t_full_q(frame)), ...
          'Color', 'w', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X (m)', 'Color', 'w'); ylabel('Y (m)', 'Color', 'w'); zlabel('Z (m)', 'Color', 'w');
    set(gca, 'Color', [0.15 0.15 0.2], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    xlim([-max_reach max_reach]); ylim([-max_reach max_reach]); zlim([-0.2 max_reach+0.5]);
    axis equal; grid on;
    set(gca, 'GridColor', [0.3 0.3 0.4]);
    view(45, 25);
    
    drawnow;
    pause(0.02);
end

fprintf('>> Trajectory Planning Complete!\n');
fprintf('══════════════════════════════════════════════════════\n');

%% ======================== HELPER FUNCTIONS ==============================

function T = dh_transform(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

function [pos, vel, acc] = cubic_trajectory(q0, qf, tf, t)
    % Cubic polynomial: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    % Boundary conditions: q(0)=q0, q(tf)=qf, qd(0)=0, qd(tf)=0
    
    a0 = q0;
    a1 = 0;
    a2 = 3*(qf - q0) / tf^2;
    a3 = -2*(qf - q0) / tf^3;
    
    pos = a0 + a1*t + a2*t.^2 + a3*t.^3;
    vel = a1 + 2*a2*t + 3*a3*t.^2;
    acc = 2*a2 + 6*a3*t;
end

function [pos, vel, acc] = quintic_trajectory(q0, qf, tf, t)
    % Quintic polynomial: q(t) = a0 + a1*t + ... + a5*t^5
    % Boundary conditions: q(0)=q0, q(tf)=qf
    %                      qd(0)=0, qd(tf)=0
    %                      qdd(0)=0, qdd(tf)=0
    
    a0 = q0;
    a1 = 0;
    a2 = 0;
    a3 = 10*(qf - q0) / tf^3;
    a4 = -15*(qf - q0) / tf^4;
    a5 = 6*(qf - q0) / tf^5;
    
    pos = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
    vel = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
    acc = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
end
