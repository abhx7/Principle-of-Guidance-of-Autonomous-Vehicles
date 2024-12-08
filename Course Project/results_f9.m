clc;
clear;
close all;

R_0 = 2500;
theta_0 = 0*pi/180;
x_p0 = 0;
y_p0 = 0;
x_t0 = R_0*cos(theta_0);
y_t0 = R_0*sin(theta_0);

%% TIME CONDITIONS
t_step = 0.05;
t_end = 1000;
t_span = 0:t_step:t_end;
t_terminate = 5;
options = odeset('Events', @(t, y) event_terminal(t, y));
%% PPN, 2pPPN and BPPN
%N = 3;
alpha_P_df = 5*pi/6;
V_T = 0;
V_P = 50;
alpha_P0 = 3*pi/4;
alpha_T0 = 0;
V_R0 = V_T*cos(alpha_T0 - theta_0) - V_P*cos(alpha_P0 - theta_0);
V_theta_0 = V_T*sin(alpha_T0 - theta_0) - V_P*sin(alpha_P0 - theta_0);
y0 = [R_0, theta_0, V_theta_0, V_R0, alpha_P0, alpha_T0, x_t0, y_t0, x_p0, y_p0];
%[t,y] = ode45(@(t,y) PPN_paper(t, y, V_P, V_T, alpha_P_df), t_span, y0, options);

Bias = [0.02, 0.05, 0.08, 0.1, 0.3, 0.5];
theta3 = [-0.6, -0.25, -0.095, -0.0945, -0.05, -0.02];

% Pre-allocate cell arrays to store results for different bias values
t_results = cell(length(Bias), 1);
y_results = cell(length(Bias), 1);
B_values = cell(length(Bias), 1); % For storing B for each bias

% Loop over each bias value
for i = 1:length(Bias)
    current_B = Bias(i);  % Current bias value
    current_theta_3 = theta3(i);
    
    % Solve the ODE with the current bias value
    [t1, y1] = ode45(@(t, y) BPPN_f9(t, y, V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0, current_B, current_theta_3), t_span, y0, options);
    
    % Initialize B array for this bias
    B_1 = zeros(length(t1), 1);
   
    % Compute B for each time step
    for j = 1:length(t1)
        [~, B1] = BPPN_f9(t1(j), y1(j,:), V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0, current_B, current_theta_3);
        B_1(j) = B1;
    end
    
    % Store the time, solution, and B values
    t_results{i} = t1;
    y_results{i} = y1;
    B_values{i} = B_1;
end

%% PLOTS AND ANIMATION


% Plot B vs Time
figure;
hold on;

% Loop through the results and plot them
for i = 1:length(Bias)
    % Plot normalized B vs. time
    plot(t_results{i}, (1/V_P).*B_values{i}, 'DisplayName', sprintf('Bias = %.2f', Bias(i)), 'LineWidth',1.5);
end

hold off;
legend('Location', 'best');
xlabel('Time [s]');
ylabel('Normalized Bias (B/V_P)');
title('B vs Time for Different Bias Values');
grid on;

 
% Pre-allocate arrays for storing trajectories for each bias
x_T_results = cell(length(Bias), 1);
y_T_results = cell(length(Bias), 1);
x_P_results = cell(length(Bias), 1);
y_P_results = cell(length(Bias), 1);

% Loop over each bias value to extract the corresponding trajectories
for i = 1:length(Bias)
    x_T_results{i} = y_results{i}(:, 7); % Extract x-position of the target
    y_T_results{i} = y_results{i}(:, 8); % Extract y-position of the target
    x_P_results{i} = y_results{i}(:, 9); % Extract x-position of the pursuer
    y_P_results{i} = y_results{i}(:, 10); % Extract y-position of the pursuer
end

% Plot the initial and final positions for each bias
figure;
hold on;
% Plot target's initial and final positions (should be stationary)
plot(x_T_results{1}(1), y_T_results{1}(1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'HandleVisibility', 'off'); % Target's initial position
plot(x_T_results{1}(end), y_T_results{1}(end), 'b*', 'MarkerSize', 8, 'HandleVisibility', 'off');                    % Target's final position

% Plot pursuer's initial and final positions
plot(x_P_results{1}(1), y_P_results{1}(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off'); % Pursuer's initial position
plot(x_P_results{1}(end), y_P_results{1}(end), 'r*', 'MarkerSize', 8, 'HandleVisibility', 'off');                    % Pursuer's final position

% Loop through each bias and plot the corresponding trajectories
for i = 1:length(Bias)
    % Plot the trajectories for target and pursuer
    %plot(x_T_results{i}, y_T_results{i}, '-', 'LineWidth', 1.5, 'DisplayName', sprintf('Target Bias = %.2f', Bias(i))); % Target trajectory (should be stationary)
    plot(x_P_results{i}, y_P_results{i}, '-', 'LineWidth', 1.5, 'DisplayName', sprintf('Pursuer Bias = %.2f', Bias(i))); % Pursuer trajectory
end

% Labeling the plot
xlabel('Downrange (m)');
ylabel('Crossrange (m)');
title("UAV's trajectory for different Bias values");
legend('Location', 'best');
grid on;
axis equal;
hold off;

%% %--- Event Function to Stop Simulation when Intercepting Target ---%
function [value, isterminal, direction] = event_terminal(t, y)
    R = y(1); % Range
    R0 = 2500; % Initial separation distance (same as in your initial conditions)  
    value = R - 0.01*R0; % Stop when R is 10% of R0  
    isterminal = 1; % Stop the integration
    direction = -1; % Detect when R is decreasing
end