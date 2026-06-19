%% ========================================================================
%  JACOBIAN & VELOCITY ANALYSIS - 3D ROBOTIC ARM
%  ========================================================================
%  Description : Computes the Jacobian matrix, manipulability index,
%                and performs velocity kinematics analysis
%  Author      : Sandeep Gupta
%  Platform    : MATLAB R2020a or later
%  ========================================================================

clc;
clear all;
close all;

%% Robot Parameters
L1 = 1.0;   % Link 1 length
L2 = 0.8;   % Link 2 length
L3 = 0.6;   % Link 3 length

fprintf('╔══════════════════════════════════════════════════════╗\n');
fprintf('║   JACOBIAN & VELOCITY KINEMATICS ANALYSIS          ║\n');
fprintf('╚══════════════════════════════════════════════════════╝\n\n');

%% Define Joint Configuration
theta1 = 45;   % degrees
theta2 = 30;   % degrees
theta3 = -45;  % degrees

t1 = deg2rad(theta1);
t2 = deg2rad(theta2);
t3 = deg2rad(theta3);

%% Compute Jacobian Matrix (Geometric Method)
fprintf('>> Computing Geometric Jacobian...\n\n');

% Forward Kinematics
T01 = dh_transform(t1, L1, 0, pi/2);
T02 = T01 * dh_transform(t2, 0, L2, 0);
T03 = T02 * dh_transform(t3, 0, L3, 0);

% Joint positions
O0 = [0; 0; 0];
O1 = T01(1:3, 4);
O2 = T02(1:3, 4);
O3 = T03(1:3, 4);  % End-effector

% Joint axes (z-axes of each frame)
z0 = [0; 0; 1];
z1 = T01(1:3, 3);
z2 = T02(1:3, 3);

% Geometric Jacobian (linear velocity part only for 3-DOF position)
J = [cross(z0, O3-O0), cross(z1, O3-O1), cross(z2, O3-O2)];

fprintf('Jacobian Matrix (3x3):\n');
fprintf('  J = [%.4f  %.4f  %.4f]\n', J(1,:));
fprintf('      [%.4f  %.4f  %.4f]\n', J(2,:));
fprintf('      [%.4f  %.4f  %.4f]\n\n', J(3,:));

%% Manipulability Analysis
fprintf('>> Manipulability Analysis...\n');

det_J = det(J);
fprintf('   det(J)           = %.6f\n', det_J);

if abs(det_J) < 1e-6
    fprintf('   ⚠ WARNING: Near singular configuration!\n');
else
    fprintf('   ✓ Non-singular configuration\n');
end

% Yoshikawa manipulability index
w = sqrt(abs(det(J * J')));
fprintf('   Manipulability (w)= %.6f\n', w);

% Condition number
cond_J = cond(J);
fprintf('   Condition number  = %.4f\n\n', cond_J);

%% Velocity Kinematics
fprintf('>> Velocity Kinematics Test...\n');

% Given joint velocities
q_dot = [0.5; 0.3; -0.2];  % rad/s
fprintf('   Joint velocities: [%.1f, %.1f, %.1f] rad/s\n', q_dot);

% End-effector linear velocity
v_ee = J * q_dot;
fprintf('   End-effector velocity:\n');
fprintf('     Vx = %.4f m/s\n', v_ee(1));
fprintf('     Vy = %.4f m/s\n', v_ee(2));
fprintf('     Vz = %.4f m/s\n', v_ee(3));
fprintf('     |V| = %.4f m/s\n\n', norm(v_ee));

%% Manipulability Ellipsoid Visualization
fprintf('>> Rendering Manipulability Ellipsoid...\n');

figure('Name', 'Manipulability Ellipsoid', ...
       'NumberTitle', 'off', ...
       'Color', [0.1 0.1 0.15], ...
       'Position', [100 100 1200 800]);

% SVD of Jacobian
[U, S, V] = svd(J);
singular_values = diag(S);

fprintf('   Singular values: [%.4f, %.4f, %.4f]\n', singular_values);

% Create unit sphere
[xs, ys, zs] = sphere(30);

% Transform sphere to ellipsoid
for i = 1:numel(xs)
    point = [xs(i); ys(i); zs(i)];
    transformed = J * point;
    xs(i) = transformed(1) + O3(1);
    ys(i) = transformed(2) + O3(2);
    zs(i) = transformed(3) + O3(3);
end

% Plot
surf(xs, ys, zs, 'FaceColor', [0.2 0.6 1], ...
     'FaceAlpha', 0.3, 'EdgeColor', [0.3 0.7 1], 'EdgeAlpha', 0.2);
hold on;

% Plot robot arm
joints = [O0'; O1'; O2'; O3'];
plot3(joints(:,1), joints(:,2), joints(:,3), ...
      'Color', [0.1 0.3 0.6], 'LineWidth', 6);
plot3(joints(:,1), joints(:,2), joints(:,3), ...
      'Color', [0.3 0.7 1], 'LineWidth', 3);

colors = [0.2 0.8 0.3; 1 0.8 0; 1 0.4 0.1; 1 0.2 0.3];
sizes = [100 80 70 50];
for i = 1:4
    scatter3(joints(i,1), joints(i,2), joints(i,3), ...
             sizes(i), colors(i,:), 'filled', ...
             'MarkerEdgeColor', 'white', 'LineWidth', 1.5);
end

% Principal axes of ellipsoid
scale = 0.3;
for i = 1:3
    quiver3(O3(1), O3(2), O3(3), ...
            U(1,i)*singular_values(i)*scale, ...
            U(2,i)*singular_values(i)*scale, ...
            U(3,i)*singular_values(i)*scale, ...
            'Color', colors(i,:), 'LineWidth', 2.5, 'MaxHeadSize', 0.4);
end

hold off;

title(sprintf('Manipulability Ellipsoid (w = %.4f)', w), ...
      'Color', 'white', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X (m)', 'Color', 'white', 'FontSize', 12);
ylabel('Y (m)', 'Color', 'white', 'FontSize', 12);
zlabel('Z (m)', 'Color', 'white', 'FontSize', 12);
set(gca, 'Color', [0.15 0.15 0.2], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
axis equal; grid on;
set(gca, 'GridColor', [0.3 0.3 0.4], 'GridAlpha', 0.5);
view(45, 25);

%% Manipulability Map over Joint Space
fprintf('>> Computing Manipulability Map...\n');

figure('Name', 'Manipulability Map', ...
       'NumberTitle', 'off', ...
       'Color', [0.1 0.1 0.15], ...
       'Position', [150 150 1200 500]);

N = 50;
t1_range = linspace(-pi, pi, N);
t2_range = linspace(-pi/2, pi/2, N);
w_map = zeros(N, N);

t3_fixed = deg2rad(-30);

for i = 1:N
    for j = 1:N
        T01_t = dh_transform(t1_range(i), L1, 0, pi/2);
        T02_t = T01_t * dh_transform(t2_range(j), 0, L2, 0);
        T03_t = T02_t * dh_transform(t3_fixed, 0, L3, 0);
        
        O0_t = [0;0;0]; O1_t = T01_t(1:3,4);
        O2_t = T02_t(1:3,4); O3_t = T03_t(1:3,4);
        
        z0_t = [0;0;1]; z1_t = T01_t(1:3,3); z2_t = T02_t(1:3,3);
        
        J_t = [cross(z0_t, O3_t-O0_t), cross(z1_t, O3_t-O1_t), cross(z2_t, O3_t-O2_t)];
        w_map(j, i) = sqrt(abs(det(J_t * J_t')));
    end
end

imagesc(rad2deg(t1_range), rad2deg(t2_range), w_map);
colormap(hot);
colorbar('Color', 'white');
xlabel('θ₁ (deg)', 'Color', 'white', 'FontSize', 12);
ylabel('θ₂ (deg)', 'Color', 'white', 'FontSize', 12);
title(sprintf('Manipulability Map (θ₃ = %.0f°)', rad2deg(t3_fixed)), ...
      'Color', 'white', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'Color', [0.15 0.15 0.2], 'XColor', 'w', 'YColor', 'w', ...
         'YDir', 'normal');
axis tight;

fprintf('   Map resolution: %dx%d\n', N, N);
fprintf('\n>> Jacobian Analysis Complete!\n');
fprintf('══════════════════════════════════════════════════════\n');

%% Helper Function
function T = dh_transform(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
