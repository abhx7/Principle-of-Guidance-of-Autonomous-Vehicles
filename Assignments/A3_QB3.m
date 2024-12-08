% Clear workspace and close all figures
clear;
clc;
close all;

% Define Simulation Parameters
VP = 400; % Target speed in m/s
nu = 0.6;
VT = (1/nu) * VP; % Pursuer speed in m/s (modify as needed)
e = 1e-3; % threshold to prevent division by zero

delta = deg2rad(0); % initial Deviation angle in radians
theta0 = deg2rad(30); % Initial LOS angle in radians
alpha_P0 = theta0 + delta; % Initial pursuer angle in radians
alpha_T0 = deg2rad(60); % Initial target angle in radians
R0 = 7000; % Initial separation distance in meters
T_end = 50; % End time for simulation (adjust if needed)

% Calculate initial velocities
V_R0 = VT * cos(alpha_T0 - theta0) - VP * cos(delta);
V_theta0 = VT * sin(alpha_T0 - theta0) - VP * sin(delta);

% Time vector
%tspan = [0 T_end]
n = 1000;
tspan = linspace(0, T_end, n);

% Initial conditions [R, theta, V_theta, V_R, alpha_P, VP, alpha_T, VT]
initial_conditions = [R0, theta0, V_theta0, V_R0, alpha_P0, VP, alpha_T0, VT];

% Set event function to stop integration when R = 0 (interception)
options = odeset('Events', @event_function);

%%
%------------------------- Proportional Navigation Guidance Law ------------------------%
%TPN
c = -3*V_R0; % Gain in TPN
a_T = 30; % Maneuvering parameter

[t, state, te] = ode45(@(t, y) TPN(t, y, c, a_T, 0, 0), tspan, initial_conditions, options);
%RTPN = 1;
%Nd = 2;
%[t, state, te] = ode45(@(t, y) TPN(t, y, c, b, Nd, 1), tspan, initial_conditions, options);


%---------------------------------------------------------------------------------------%

%% Extract results
R = state(:, 1); % Range
theta = state(:, 2); % LOS angle
V_theta = state(:, 3); % Angular velocity
V_R = state(:, 4); % Radial velocity
alpha_P = state(:, 5); % Pursuer heading angle
V_P = state(:, 6); % Pursuer speed
alpha_T = state(:, 7); % Target heading angle
V_T = state(:, 8); % Target speed


%%TPN
% % Calculate lateral acceleration a_P
% if RTPN == 1
%     c = -Nd*V_R;
%     a_P = c .* V_theta ./ max(R, e);
% else
%      a_P = c * V_theta ./ max(R, e);
% end
a_P = c * V_theta ./ max(R, e);


%%
% Plot results
t = plot_results(t, R, theta, V_theta, V_R, a_P, a_T, te, c, R0);

%%
% Real-time plot of the chase
plot_real_time_chase(t, R, theta, V_T, V_P, alpha_T, alpha_P);

%%
%--- Event Function to Stop Simulation when Intercepting Target ---%
function [value, isterminal, direction] = event_function(t, y)
    R = y(1); % Range
    R0 = 7000; % Initial separation distance (same as in your initial conditions)  
    value = R - 0.1 * R0; % Stop when R is 10% of R0  
    isterminal = 1; % Stop the integration
    direction = -1; % Detect when R is decreasing
end

%%
%--- Function to Plot Results ---%
function t = plot_results(t, R, theta, V_theta, V_R, aP, aT, te, c, R0)
    % Define the interception range as 10% of the initial range
    interception_range = 0.1 * R0;

    % Check if interception occurred (R <= 10% of R0)
    if ~isempty(te)
        disp(['Interception occurs for c = ', num2str(c), ' at time t = ', num2str(te), ' seconds']);
        
        % Find the index where R falls below 10% of R0
        intercept_idx = find(R <= interception_range, 1);

        if ~isempty(intercept_idx)
            % Truncate time and state values at the time of interception
            t = t(1:intercept_idx);
            R = R(1:intercept_idx);
            theta = theta(1:intercept_idx);
            V_theta = V_theta(1:intercept_idx);
            V_R = V_R(1:intercept_idx);
            aP = aP(1:intercept_idx);
            aT = aT(1:intercept_idx);
        end
    else
        % No interception - Find the miss distance (min R)
        [R_miss, idx] = min(R);
        tmiss = t(idx);
        disp(['No interception for c = ', num2str(c), '. Miss distance = ', num2str(R_miss), ' meters at t = ', num2str(tmiss), ' seconds']);
    end

    % Combined plot: R, theta, V_theta, V_R
    figure;
    subplot(2, 2, 1); plot(t, R); xlabel('Time (s)'); ylabel('Range (m)'); title('Range R');
    subplot(2, 2, 2); plot(t, theta); xlabel('Time (s)'); ylabel('\theta (rad)'); title('LOS Angle \theta');
    subplot(2, 2, 3); plot(t, V_theta); xlabel('Time (s)'); ylabel('V_\theta (rad/s)'); title('Angular Velocity V_\theta');
    subplot(2, 2, 4); plot(t, V_R); xlabel('Time (s)'); ylabel('V_R (m/s)'); title('Radial Velocity V_R');
    sgtitle(['Combined Plots for c = ', num2str(c)]);

    % Plot lateral acceleration aP 
    figure;
    plot(t, aP); xlabel('Time (s)'); ylabel('a_P (m/s^2)'); title('Lateral Acceleration a_P');
    grid on;

    % Plot lateral acceleration aT
    figure;
    plot(t, aT); xlabel('Time (s)'); ylabel('a_T (m/s^2)'); title('Lateral Acceleration a_T');
    grid on;

    % Plot (V_theta, V_R) in velocity space 
    figure;
    plot(V_theta, V_R); xlabel('V_\theta (rad/s)'); ylabel('V_R (m/s)'); title('(V_\theta, V_R) Space');
    grid on;
end


%%
%--- Function for Real-Time Plotting of the Chase ---%
function plot_real_time_chase(t, R, theta, V_T, V_P, alpha_T, alpha_P)
    % Initialize positions of the pursuer and target
    pursuer_pos = [0, 0]; % Initial pursuer position at origin
    target_pos = [R(1) * cos(theta(1)), R(1) * sin(theta(1))]; % Initial target position

    % Arrays to store trajectory history
    pursuer_traj = pursuer_pos;
    target_traj = target_pos;
   
    % Create figure for real-time plotting
    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Real-Time Pursuer and Target Trajectory');
    
    % Initialize plot handles for pursuer and target
    hPursuer = plot(pursuer_pos(1), pursuer_pos(2), 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Pursuer');
    hTarget = plot(target_pos(1), target_pos(2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Target');
    hPursuerTraj = plot(pursuer_traj(:,1), pursuer_traj(:,2), 'b--', 'DisplayName', 'Pursuer Trajectory');
    hTargetTraj = plot(target_traj(:,1), target_traj(:,2), 'r-', 'DisplayName', 'Target Trajectory');
    legend('show');
    
    % Loop through time steps for real-time updates
    for i = 2:length(t)
        % Update target position based on constant velocity
        dt = t(i) - t(i-1); % Time step
        target_pos = target_pos + [V_T(i) * cos(alpha_T(i)),  V_T(i) * sin(alpha_T(i))] * dt;

        % Update pursuer position based on the velocity and time step
        %pursuer_pos = pursuer_pos + [VP * cos(alpha_P(i)), VP * sin(alpha_P(i))] * dt;
        pursuer_pos = target_pos - [R(i) * cos(theta(i)),  R(i) * sin(theta(i))];

        % Append current positions to trajectory history
        pursuer_traj = [pursuer_traj; pursuer_pos];
        target_traj = [target_traj; target_pos];

        % Ensure plot handles are valid before updating
        if isvalid(hPursuer) && isvalid(hTarget) && isvalid(hPursuerTraj) && isvalid(hTargetTraj)
            % Update plot positions and trajectories
            set(hPursuer, 'XData', pursuer_pos(1), 'YData', pursuer_pos(2));
            set(hTarget, 'XData', target_pos(1), 'YData', target_pos(2));
            set(hPursuerTraj, 'XData', pursuer_traj(:,1), 'YData', pursuer_traj(:,2));
            set(hTargetTraj, 'XData', target_traj(:,1), 'YData', target_traj(:,2));

            % Pause to create real-time effect
            pause(1e-6);
        else
            % Exit loop if plot handles are no longer valid (e.g., figure closed)
            break;
        end
    end
end

function dydt = TPN(t, y, c, a_T, Nd, RTPN)
    % Extract state variables
    R = y(1);           % Range
    theta = y(2);        % Line-of-sight angle (theta)
    V_theta = y(3);      % Angular velocity (V_theta)
    V_R = y(4);          % Radial velocity (V_R)
    alpha_P = y(5);      % Pursuer heading angle (alpha_P)
    V_P = y(6);          % Pursuer speed (V_P, allows for dynamic updates)
    alpha_T = y(7);      % Target heading angle (alpha_T)
    V_T = y(8);          % Target speed (VT, allows for dynamic updates)


    % Compute LOS rate and relative velocity components
    V_R = V_T * cos(alpha_T - theta) - V_P * cos(alpha_P - theta);
    V_theta = V_T * sin(alpha_T - theta) - V_P * sin(alpha_P - theta);
    
    if RTPN == 1
        c = - Nd *V_R; %RTPN
    end

    % PN law: a_P = N_c * |dtheta/dt|
    if R ~= 0
        a_P = c * abs(V_theta / R);  % Pursuer's lateral acceleration using PN
    else
        a_P = c * abs(V_theta /1e-3);  % Prevent division by zero
    end
    

    % Time derivatives
    dR_dt = V_R;                                % Range rate
    dtheta_dt = V_theta / R;                    % LOS angle rate
    dV_R_dt = dtheta_dt * V_theta;              % Radial velocity rate
    dV_theta_dt = -dtheta_dt * V_R + a_T - a_P; % Angular velocity rate
    dalpha_P_dt = a_P * cos(alpha_P - theta);   % Pursuer heading angle rate
    dVP_dt = a_P * sin(alpha_P - theta);        % Pursuer's velocity rate
    % Target dynamics
    dalpha_T_dt = a_T * cos(alpha_T - theta)/V_T;   % Target heading angle rate
    dVT_dt = a_T * sin(alpha_T - theta);        % Target velocity rate

    % Return derivatives [dR/dt, dtheta/dt, dV_R/dt, dV_theta/dt, dalpha_P/dt, dVP/dt, dalpha_T/dt, dVT/dt]
    dydt = [dR_dt; dtheta_dt; dV_R_dt; dV_theta_dt; dalpha_P_dt; dVP_dt; dalpha_T_dt; dVT_dt];
end
