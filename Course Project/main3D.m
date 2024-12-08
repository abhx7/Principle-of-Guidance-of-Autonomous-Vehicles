% Clear workspace and close all figures
clear;
clc;
close all;

% Define Simulation Parameters
VT = 0; % Target speed in m/s
VP = 50; % Pursuer speed in m/s
nu = VP / VT; % Speed ratio

alpha_P_d_f = 5 * pi / 6; % Desired final heading angle in radians
disp(['Desired alpha_P_d_f = ', num2str(rad2deg(alpha_P_d_f))]);

e = 1e-3; % Tolerance

delta0 = deg2rad(135); % Initial deviation angle in radians
theta0 = deg2rad(0); % Initial LOS angle in radians
phi0 = deg2rad(0); % Initial LOS angle in radians
alpha_P0 = theta0 + delta0; % Initial pursuer angle in radians
disp(['alpha_P0 = ', num2str(rad2deg(alpha_P0))]);
alpha_T0 = deg2rad(60); % Initial target angle in radians

% Define initial heading angles (in radians)
alpha_P_el0 = deg2rad(5);  % Example elevation angle for the pursuer
alpha_T_el0 = deg2rad(0);  % Example elevation angle for the target

R0 = 2500; % Initial separation distance in meters
T_end = 100; % End time for simulation
n = 2000;
tspan = linspace(0, T_end, n); % Time span for the simulation

% Initial angular velocities
V_theta0 = VT * sin(alpha_T0 - theta0) - VP * sin(alpha_P0 - theta0); % Initial angular velocity in azimuth
V_R0 = VT * cos(alpha_T0 - theta0) - VP * cos(alpha_P0 - theta0);     % Initial radial velocity
V_phi0 = VT * sin(alpha_T_el0 - phi0) - VP * sin(alpha_P_el0 - phi0);   % Initial angular velocity in elevation

% Initialize state vector
initial_conditions =[R0; theta0; phi0; V_theta0; V_R0; V_phi0; alpha_P0; alpha_P_el0; alpha_T0; alpha_T_el0];

% Set event function to stop integration when R is sufficiently small
options = odeset('Events', @(t, y) event_function(t, y, R0));

% Run the simulation
[t, state, te] = ode45(@(t, y) TwopPPN_3D(t, y, VP, VT, 0, alpha_P_d_f, alpha_P0, theta0), tspan, initial_conditions, options);

% Extract results
R = state(:, 1); % Range
theta = state(:, 2); % LOS angle (azimuth)
phi = state(:, 3); % Elevation angle
V_theta = state(:, 4); % Angular velocity (azimuth)
V_R = state(:, 6); % Radial velocity
alpha_P_az = state(:, 7); % Pursuer azimuth angle
alpha_T_az = state(:, 9); % Target azimuth angle

% Calculate positions in 3D (x, y, z coordinates)
x_pursuer = R .* cos(theta) .* cos(phi);
y_pursuer = R .* sin(theta) .* cos(phi);
z_pursuer = R .* sin(phi);

% Plot Results
plot_results_3D(t, R, theta, phi, V_theta, V_R, te, R0, alpha_P0);

% Real-time 3D Plot of the Chase
plot_real_time_chase_3D(t, R, theta, phi, VT * ones(length(t)), VP, alpha_T0, alpha_P0);

% --- Event Function to Stop Simulation when Intercepting Target ---
function [value, isterminal, direction] = event_function(~, y, R0)
    R = y(1); % Range 
    value = R - 0.01 * R0; % Stop when R is 1% of R0 
    isterminal = 1; % Stop the integration 
    direction = -1; % Detect when R is decreasing 
end 

% --- Function to Plot Results in 3D --- 
function plot_results_3D(t, R, theta, phi, V_theta, V_R, te, R0, alpha_P0) 
    % Define interception range
    interception_range = 0.01 * R0;

    % Check for interception or miss 
    if ~isempty(te) 
        disp(['Interception occurs at t = ', num2str(te), ' seconds']); 

        % Find the index where R falls below 10% of R0 
        intercept_idx = find(R < interception_range, 1); 
        if ~isempty(intercept_idx) 
            % Truncate time and state values at interception 
            t = t(1:intercept_idx); 
            R = R(1:intercept_idx); 
            theta = theta(1:intercept_idx); 
            phi = phi(1:intercept_idx);
            V_theta = V_theta(1:intercept_idx); 
            V_R = V_R(1:intercept_idx); 
            disp(['Final intercept angle = ', num2str(rad2deg(alpha_P0)), '°']);
        end 
    else 
        % No interception 
        [R_miss, idx] = min(R); 
        tmiss = t(idx); 
        disp(['No interception. Miss distance = ', num2str(R_miss), ' meters at t = ', num2str(tmiss), ' seconds']); 
    end 

    % Plot trajectory in 3D 
    [x, y, z] = sph2cart(theta, phi, R);
    figure(1); 
    plot3(x, y, z); xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Z Position (m)');
    title('3D Trajectory of Pursuer'); grid on;

    % Combined plots for R, theta, V_theta, V_R 
    figure(2); 
    subplot(2, 2, 1); plot(t, R); xlabel('Time (s)'); ylabel('Range (m)'); title('Range R');
    subplot(2, 2, 2); plot(t, theta); xlabel('Time (s)'); ylabel('\theta (rad)'); title('LOS Angle \theta');
    subplot(2, 2, 3); plot(t, V_theta); xlabel('Time (s)'); ylabel('V_\theta (rad/s)'); title('Angular Velocity V_\theta');
    subplot(2, 2, 4); plot(t, V_R); xlabel('Time (s)'); ylabel('V_R (m/s)'); title('Radial Velocity V_R');
    sgtitle('Combined Plots');
end 

%--- Function for Real-Time Plotting of the Chase in 3D ---
function plot_real_time_chase_3D(t, R, theta, phi, V_T, V_P, alpha_T, alpha_P)
    % Initialize positions in 3D for the pursuer and target
    pursuer_pos = [0, 0, 0]; % Initial pursuer position at origin
    target_pos = [R(1) * cos(theta(1)) * cos(phi(1)), R(1) * sin(theta(1)) * cos(phi(1)), R(1) * sin(phi(1))]; % Initial target position in 3D

    % Arrays to store trajectory history
    pursuer_traj = pursuer_pos;
    target_traj = target_pos;

    % Create figure for real-time 3D plotting
    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    title('Real-Time Pursuer and Target 3D Trajectory');

    % Initialize plot handles for pursuer and target
    hPursuer = plot3(pursuer_pos(1), pursuer_pos(2), pursuer_pos(3), 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Pursuer');
    hTarget = plot3(target_pos(1), target_pos(2), target_pos(3), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Target');
    hPursuerTraj = plot3(pursuer_traj(:,1), pursuer_traj(:,2), pursuer_traj(:,3), 'b--', 'DisplayName', 'Pursuer Trajectory');
    hTargetTraj = plot3(target_traj(:,1), target_traj(:,2), target_traj(:,3), 'r-', 'DisplayName', 'Target Trajectory');
    legend;

    % Loop through time to update positions
    for i = 1:length(t)
        % Update pursuer position
        pursuer_pos = [R(i) * cos(theta(i)) * cos(phi(i)), R(i) * sin(theta(i)) * cos(phi(i)), R(i) * sin(phi(i))];
        pursuer_traj = [pursuer_traj; pursuer_pos]; % Append new position to trajectory history
        
        % Update target position based on its velocity
        target_pos = target_pos + [V_T(i) * cos(alpha_T), V_T(i) * sin(alpha_T), 0]; % Update target position
        target_traj = [target_traj; target_pos]; % Append new position to trajectory history
        
        % Update plot handles
        set(hPursuer, 'XData', pursuer_pos(1), 'YData', pursuer_pos(2), 'ZData', pursuer_pos(3));
        set(hTarget, 'XData', target_pos(1), 'YData', target_pos(2), 'ZData', target_pos(3));
        set(hPursuerTraj, 'XData', pursuer_traj(:,1), 'YData', pursuer_traj(:,2), 'ZData', pursuer_traj(:,3));
        set(hTargetTraj, 'XData', target_traj(:,1), 'YData', target_traj(:,2), 'ZData', target_traj(:,3));
        
        % Pause to create a real-time effect
        pause(0.05);
    end
    
    hold off;
end 
