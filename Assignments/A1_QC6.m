clc;
clear all;
close all;

% Constants
VT = 300; % Target speed in m/s
nu_values = [0.8, 1.5, 3]; % Speed ratios VP/VT
theta0 = deg2rad(30); % Initial LOS angle in radians
alpha_P0 = theta0; % Initial pursuer angle in radians
alpha_T0 = deg2rad(170); % Initial target angle in radians
R0 = 3000; % Initial separation distance in meters
T_end = 50; % End time for simulation (adjust if needed)
cT_values = [0.8, 1, 1.5] ; % Maneuvering rate (set manually)

% Time vector
tspan = [0 T_end];
%tspan = linspace(0,T_end,1000);

% Loop through each speed ratio
for i = 1:length(cT_values)
    cT = cT_values(i);
    for j = 1:length(nu_values)
        nu = nu_values(j);
        VP = nu * VT;
        
        % Calculate initial velocities
        V_R0 = VT * cos(alpha_T0 - theta0) - VP;
        V_theta0 = VT * sin(alpha_T0 - theta0);
        
        % Initial conditions [R, theta, V_theta, V_R]
        initial_conditions = [R0, theta0, V_theta0, V_R0];
    
        % Set event function to stop integration when R = 0 (interception)
        options = odeset('Events', @event_function);
    
        % Solve the differential equations using ode45
        [t, state, te, ye, ie] = ode45(@(t, y) PP(t, y, VP, VT, alpha_T0, cT), tspan, initial_conditions, options);
    
        % Extract results
        R = state(:, 1); % Range
        theta = state(:, 2); % LOS angle
        V_theta = state(:, 3); % Angular velocity
        V_R = state(:, 4); % Radial velocity
        
        % Calculate lateral acceleration
        e = 1e-3;
        aP = VP * V_theta ./ max(R, e);
        
        alpha_T = alpha_T0 + cT * theta;
        
        % Plot results
        t = plot_results(t, R, theta, V_theta, V_R, aP, te, cT);
        
        % Real-time plot of the chase
        plot_real_time_chase(t, R, theta, VP, VT, alpha_T);
    end
end

%%--- Event Function to Stop Simulation when Intercepting Target ---%
function [value, isterminal, direction] = event_function(t, y)
    R = y(1); % Range
    value = R - 1; % Stop when R is very close to zero
    isterminal = 1; % Stop the integration
    direction = -1; % Detect when R is decreasing
end

%%
%--- Function to Plot Results ---%

function t=plot_results(t, R, theta, V_theta, V_R, aP, te, cT)
    % Check if interception occurred
    if ~isempty(te)
        disp(['Interception occurs for c_T = ', num2str(cT), ' at time t = ', num2str(te), ' seconds']);
        % Truncate time and state values at the time of interception
        t = t(t <= te);
        R = R(t <= te);
        theta = theta(t <= te);
        V_theta = V_theta(t <= te);
        V_R = V_R(t <= te);
        aP = aP(t <= te);
    else
        [R_miss, idx] = min(R);
        tmiss = t(idx);
        disp(['No interception for c_T = ', num2str(cT), '. Miss distance = ', num2str(R_miss), ' meters at t = ', num2str(tmiss), ' seconds']);
    end

    % Combined plot: R, theta, V_theta, V_R
    figure;
    subplot(2, 2, 1); plot(t, R); xlabel('Time (s)'); ylabel('Range (m)'); title('Range R');
    subplot(2, 2, 2); plot(t, theta); xlabel('Time (s)'); ylabel('\theta (rad)'); title('LOS Angle \theta');
    subplot(2, 2, 3); plot(t, V_theta); xlabel('Time (s)'); ylabel('V_\theta (rad/s)'); title('Angular Velocity V_\theta');
    subplot(2, 2, 4); plot(t, V_R); xlabel('Time (s)'); ylabel('V_R (m/s)'); title('Radial Velocity V_R');
    sgtitle(['Combined Plots for c_T = ', num2str(cT)]);

    % Plot lateral acceleration aP 
    figure;
    plot(t, aP); xlabel('Time (s)'); ylabel('a_P (m/s^2)'); title('Lateral Acceleration a_P');
    grid on;

    % Plot (V_theta, V_R) in velocity space 
    figure;
    plot(V_theta, V_R); xlabel('V_\theta (rad/s)'); ylabel('V_R (m/s)'); title('(V_\theta, V_R) Space');
    grid on;
end

%%
%--- Function for Real-Time Plotting of the Chase ---%
function plot_real_time_chase(t, R, theta, VP, VT, alpha_T)
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
        target_pos = target_pos + VT * [ cos(alpha_T(i)), sin(alpha_T(i))] * dt;
         
        % Pursuer direction (theta )
        alpha_P = theta(i);
        % Update pursuer position based on the velocity and time step
        pursuer_pos = target_pos - [R(i) * cos(theta(i)),  R(i) * sin(theta(i))];
        %pursuer_pos = pursuer_pos +  VP * [cos(alpha_P), sin(alpha_P)] * dt;

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
            pause(0.01);
        else
            % Exit loop if plot handles are no longer valid (e.g., figure closed)
            break;
        end
    end
end

function dydt = PP(t, y, VP, VT, alpha_T0, cT)
    % Extract state variables
    R = y(1); % Range
    theta = y(2); % Line-of-sight angle (theta)
    V_theta = y(3); % Angular velocity (V_theta)
    V_R = y(4); % Radial velocity (V_R)
    
    % Target heading angle (alpha_T)
    alpha_T = alpha_T0 + cT * theta; % Maneuvering target
    % cT=0 for Constant target heading
    
    % Radial velocity and angular velocity equations for deviated pure pursuit
    V_R = VT * cos(alpha_T - theta) - VP;
    V_theta = VT * sin(alpha_T - theta);
    
    % Time derivatives
    dR_dt = V_R; % Range rate
    dtheta_dt = V_theta / R; % LOS angle rate
    dV_R_dt = dtheta_dt * (V_theta); 
    dV_theta_dt = -dtheta_dt * (V_R + VP); 
    
    % Return the derivatives
    dydt = [dR_dt; dtheta_dt; dV_R_dt; dV_theta_dt];
end
