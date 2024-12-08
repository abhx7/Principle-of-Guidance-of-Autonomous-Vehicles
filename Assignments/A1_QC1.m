% MATLAB Code to plot Range R vs Angle psi for different speed ratios

% Given values
R0 = 5000; % Initial range in meters
psi0 = pi / 3; % Initial angle in radians
nu_values = [0.8, 1, 1.2]; % Speed ratios

% Define psi range from 0 to pi, monotonic 
psi = linspace(0, 0.8*pi, 100);

% Create a figure for plotting
figure(1);
hold on;

% Loop over different speed ratios
for nu = nu_values
    % Compute the range R as a function of psi
    R = R0 * (power(cos(psi0/2),nu+1))/(power(sin(psi0/2),nu-1)) * (power(sin(psi/2),nu-1))./(power(cos(psi/2),nu+1));
    
    % Plot R vs psi
    plot(psi, R, 'DisplayName', sprintf('\\nu = %.1f', nu));
end

% Add labels, title, and legend
xlabel('\psi (radians)');
ylabel('Range R (meters)');
title('Variation of Range R with \psi for Different Speed Ratios');
legend('show');
grid on;
hold off;


% % Parameters
% R0 = 5000; % Initial range in meters
% psi0 = pi/3; % Initial angle in radians
% alpha_P0 = psi0; % Initial heading angle
% V_T = 300; % Target speed in m/s (example value)
% nu = [0.8, 1, 1.2]; % Speed ratios
% 
% % Preallocate range arrays
% R_values = zeros(100, length(nu));
% psi_values = linspace(0, 2*pi, 100);
% 
% % Loop over different speed ratios
% for j = 1:length(nu)
%     V_P = nu(j) * V_T; % Pursuer speed
%     for i = 1:length(psi_values)
%         psi = psi_values(i);
%         % Calculate range R as a function of psi (example function)
%         R_values(i, j) = R0 * exp(-psi); % Simplified for demonstration
%     end
% end
% 
% % Plot results
% figure;
% hold on;
% plot(psi_values, R_values(:, 1), 'r-', 'DisplayName', '\nu = 0.8');
% plot(psi_values, R_values(:, 2), 'b-', 'DisplayName', '\nu = 1');
% plot(psi_values, R_values(:, 3), 'g-', 'DisplayName', '\nu = 1.2');
% xlabel('\psi (radians)');
% ylabel('Range R (meters)');
% title('Variation of Range R with \psi for different speed ratios \nu');
% legend show;
% grid on;
