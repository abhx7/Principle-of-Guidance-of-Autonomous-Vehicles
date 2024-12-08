% Define parameters
gamma_P = 5; % Pursuer's angular rate (example value)
gamma_T = 0.5; % Target's angular rate (example value)
V_P = 2; % Pursuer's speed
V_T = 1; % Target's speed
R_P = V_P / gamma_P;
R_T = V_T / gamma_T;

% Define time vector
t = linspace(0, 20, 1000); % Adjust time range as needed

% Position functions for P and T (outputs as arrays of positions over time)
X_P = @(t) [R_P * cos(gamma_P * t); R_P * sin(gamma_P * t)];
X_T = @(t) [R_P + R_T * cos(gamma_T * t); R_T * sin(gamma_T * t)];

% Pre-allocate array for range
R = zeros(1, length(t));

% Calculate range over time
for i = 1:length(t)
    % Get positions at time t(i)
    pos_P = X_P(t(i));
    pos_T = X_T(t(i));
    
    % Calculate the range (distance) between pursuer and target
    R(i) = sqrt((pos_P(1) - pos_T(1))^2 + (pos_P(2) - pos_T(2))^2);
end

% Find minimum range (miss distance) and corresponding time
[R_miss, idx] = min(R);
t_miss = t(idx);

% Display results
fprintf('Miss distance: %.2f\n', R_miss);
fprintf('Time at miss distance: %.2f\n', t_miss);

% Plot positions and range over time
figure;
hold on;

% Calculate positions for plotting
P_pos = X_P(t); % Pursuer positions over time
T_pos = X_T(t); % Target positions over time

plot(P_pos(1,:), P_pos(2,:), 'b'); % Plot pursuer path
plot(T_pos(1,:), T_pos(2,:), 'r'); % Plot target path

% Mark miss point
plot(P_pos(1, idx), P_pos(2, idx), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); 

% Mark initial and final points
plot(P_pos(1,1), P_pos(2,1), 'bx', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Pursuer Start');
plot(P_pos(1,end), P_pos(2,end), 'b*', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Pursuer End');
plot(T_pos(1,1), T_pos(2,1), 'rx', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Target Start');
plot(T_pos(1,end), T_pos(2,end), 'r*', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Target End');


xlabel('x position');
ylabel('y position');
legend('Pursuer Path', 'Target Path', 'Miss Point');
title('Paths and Miss Point');
grid on;
