% Constants
VT = 100; % Target speed in m/s
nu_values = [0.8, 1, 1.5]; % Speed ratios VP/VT
K_values = [1, 5, 10]; % Maneuvering rate
%nu_values = 0.5;

alpha_P0 = deg2rad(30); % Initial pursuer angle in radians
alpha_T0 = deg2rad(170); % Initial target angle in radians
theta0 = alpha_T0 - alpha_P0; % Initial LOS angle in radians
R0 = 5000; % Initial separation distance in meters

T_end = 100; % End time for simulation (adjust if needed)
maneuvering_case = true; % Set to true if target is maneuvering, else false

% Time vector
tspan = [0 T_end];

% Loop through each speed ratio
for i = 1:length(K_values)
    for j = 1:length(nu_values)
        k = K_values(i);
        nu = nu_values(j);
        VP = nu * VT; % Pursuer speed
        
        VP_R0 = VP * cos(alpha_P0 - theta0);
        VT_R0 = VT * cos(alpha_T0 - theta0); 
        VP_theta0 = VP * sin(alpha_P0 - theta0);
        VT_theta0 = VT * sin(alpha_T0 - theta0);
        % Initial conditions
        % [R, theta, V_theta, V_R]
        initial_conditions = [R0, theta0,  VP_R0, VT_R0, VP_theta0, VT_theta0]; % Example initial R and theta, adjust as needed
        
        % Set event function to stop integration when R = 0 (interception)
        options = odeset('Events', @event_function);
        
        % Solve the differential equations using ode45
        [t, state, te, ye, ie] = ode45(@(t, y) dp_guidance_ode(t, y, VP, VT, delta, alpha_T0, maneuvering_case), tspan, initial_conditions, options);
        
        % Extract the results
        R = state(:, 1); % Range
        theta = state(:, 2); % LOS angle
        V_theta = state(:, 3); % Angular velocity
        V_R = state(:, 4); % Radial velocity
        
        % Calculate lateral acceleration aP = VP * V_theta / R, avoiding division by small R
        small_R_threshold = 1e-3; % Define a small value to avoid division by very small R
        aP = VP * V_theta ./ max(R, small_R_threshold); % Use max to avoid dividing by small R
        
        % Check if interception occurred
        if ~isempty(te) % If te is not empty, interception occurred
            disp(['Interception occurs for \nu = ', num2str(nu), ', c_T = ', num2str(cT), ' at time t = ', num2str(te), ' seconds']);
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
            disp(['No interception for \nu = ', num2str(nu), ', c_T = ', num2str(cT), '. Miss distance = ', num2str(R_miss), ' meters at t = ', num2str(tmiss), ' seconds']);
        end
        
        % Combined plot: R, theta, V_theta, V_R
        figure;
        subplot(2, 2, 1); plot(t, R); xlabel('Time (s)'); ylabel('Range (m)'); title(['Range R for \nu = ', num2str(nu), ', c_T = ', num2str(cT)]);
        subplot(2, 2, 2); plot(t, theta); xlabel('Time (s)'); ylabel('\theta (rad)'); title(['LOS Angle \theta for \nu = ', num2str(nu), ', c_T = ', num2str(cT)]);
        subplot(2, 2, 3); plot(t, V_theta); xlabel('Time (s)'); ylabel('V_\theta (rad/s)'); title(['Angular Velocity V_\theta for \nu = ', num2str(nu), ', c_T = ', num2str(cT)]);
        subplot(2, 2, 4); plot(t, V_R); xlabel('Time (s)'); ylabel('V_R (m/s)'); title(['Radial Velocity V_R for \nu = ', num2str(nu), ', c_T = ', num2str(cT)]);
        sgtitle(['Combined Plots for \nu = ', num2str(nu), ', c_T = ', num2str(cT)]);
        
        % Plot lateral acceleration aP 
        figure;
        plot(t, aP); xlabel('Time (s)'); ylabel('a_P (m/s^2)'); title(['Lateral Acceleration a_P for \nu = ', num2str(nu), ', c_T = ', num2str(cT)]);
        grid on;
        
        % Plot (V_theta, V_R) in velocity space 
        figure;
        plot(V_theta, V_R); xlabel('V_\theta (rad/s)'); ylabel('V_R (m/s)'); title(['(V_\theta, V_R) Space for \nu = ', num2str(nu), ', c_T = ', num2str(cT)]);
        grid on;
    end
end

%ned to edit and comment full part proeprly
% Function for Deviated Pure Pursuit guidance ODEs
function dydt = _guidance_ode(t, y, VP, VT, delta, alpha_T0, maneuvering_case, cT)
    % Extract state variables
    R = y(1); % Range
    theta = y(2); % Line-of-sight angle (theta)
    V_theta = y(3); % Angular velocity (V_theta)
    V_R = y(4); % Radial velocity (V_R)
    
    % Target heading angle (alpha_T)
    if maneuvering_case
        alpha_T = alpha_T0 + cT * theta; % Maneuvering target
        %alpha_T = alpha_T0 + 0.02 * sin(0.1 * t); % Example of a maneuvering target (sine wave)

    else
        alpha_T = alpha_T0; % Constant target heading
    end
    
    % Radial velocity and angular velocity equations for deviated pure pursuit
    V_R = VT * cos(alpha_T - theta) - VP * cos(delta);
    V_theta = VT * sin(alpha_T - theta) - VP * sin(delta);
    
    % Time derivatives
    dR_dt = V_R; % Range rate
    dtheta_dt = V_theta / R; % LOS angle rate
    dV_theta_dt = dtheta_dt * (V_theta + VP * sin(delta)); 
    dV_R_dt = dtheta_dt * (V_R + VP * cos(delta)); 
    
    % Return the derivatives
    dydt = [dR_dt; dtheta_dt; dV_theta_dt; dV_R_dt];
end

% Event function to stop integration when R = 0
function [value, isterminal, direction] = event_function(t, y)
    R = y(1); % Range
    value = R - 1e-3; % Stop when R is very close to zero
    isterminal = 1; % Stop the integration
    direction = -1; % Detect when R is decreasing
end
