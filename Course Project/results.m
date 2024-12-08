clc;
clear;
close all;
clear BPPN_new;
clear BPPN_without_mu;
clear BPPN;

R_0 = 2500;
theta_0 = 0*pi/180;
x_p0 = 0;
y_p0 = 0;
x_t0 = R_0*cos(theta_0);
y_t0 = R_0*sin(theta_0);

%% TIME CONDITIONS
t_step = 1e-3;
t_end = 100;
t_span = 0:t_step:t_end;
t_terminate = 5;
options = odeset('Events', @(t, y) event_terminal(t, y), 'MaxStep', 0.05, 'Refine',1);
% Set a small maximum step size
%% PPN, 2pPPN and BPPN
%N = 3;
alpha_P_df = 5*pi/6;
V_T = 5;
V_P = 50;
alpha_P0 = 3*pi/4;
alpha_T0 = 0.1;
V_R0 = V_T*cos(alpha_T0 - theta_0) - V_P*cos(alpha_P0 - theta_0);
V_theta_0 = V_T*sin(alpha_T0 - theta_0) - V_P*sin(alpha_P0 - theta_0);
y0 = [R_0, theta_0, V_theta_0, V_R0, alpha_P0, alpha_T0, x_t0, y_t0, x_p0, y_p0];
%[t,y] = ode45(@(t,y) PPN_paper(t, y, V_P, V_T, alpha_P_df), t_span, y0, options);

[t1,y1] = ode45(@(t,y) BPPN_without_mu(t, y, V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0), t_span, y0, options);
%t_span = 0:t_step*1e-2:t_end;
%[t2,y2] = ode45(@(t,y) BPPN_new(t, y, V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0), t_span, y0, options);
[t2,y2] = ode45(@(t,y) BPPN_new_new(t, y, V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0), t_span, [y0 0], options);

alphaP=y1(:, 5)*180/pi;
fprintf('alpha_P_f = %.3f \n',  alphaP(end));
alphaP=y2(:, 5)*180/pi;
fprintf('mu alpha_P_f = %.3f',  alphaP(end));


% [t1,y1] = simple_fixed_step_ode45_with_events(@(t,y) BPPN_without_mu(t, y, V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0), t_span, y0, options);
% [t2,y2] = simple_fixed_step_ode45_with_events(@(t,y) BPPN_new(t, y, V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0), t_span, y0, options);

%%
% % B_1 = zeros(length(t1) - 1, 1);
% % B_2 = zeros(length(t2) - 1, 1);
% % for i = 1:length(t1)
% %     [~, B1] = BPPN_without_mu(t1(i), y1(i,:), V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0);
% %     B_1(i) = B1;
% % end
% % 
% % for i = 1:length(t2)
% %     [~, B2] = BPPN_new(t2(i), y2(i,:), V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0);
% %     B_2(i) = B2;
% % end
% length(B_1)
% length(B_2)
% length(t_span)
% length(t1)
% length(t2)
%% PLOTS AND ANIMATION

% figure;
% plot(t, y(:, 1));
% xlabel('Time (s)');
% ylabel('R');
% 
% title('R over time');
% grid on;
% 
figure;
plot(t1, y1(:, 2),t2, y2(:, 2));
xlabel('Time (s)');
ylabel('\theta');
title('\theta over time');
legend('show');
grid on;

% figure;
% plot(t, y(:, 3));
% xlabel('Time (s)');
% ylabel('V_{\theta}');
% title('V_{\theta} over time');
% grid on;
% 
% figure;
% plot(t, y(:, 4));
% xlabel('Time (s)');
% ylabel('V_R');
% title('V_R over time');
% grid on;
% 
% figure;
% plot(t, y(:, 5));
% xlabel('Time (s)');
% ylabel('\alpha_P');
% title('\alpha_P over time');
% grid on;
% 
% 
% figure;
% plot(y(:, 3), y(:, 4));
% xlabel('V_{\theta}');
% ylabel('V_R');
% title('V_R vs V_{\theta}');
% grid on;
% 
%] N = (alpha_P_df - y(:, 5))./(alpha_P_df - y(:, 2));
% aP = (V_P/0.01).*diff(y(: , 5)); % PPN
% 
% theta_dot = y(:, 3)./y(:, 1);
% figure;
% plot(t, (180/pi).*theta_dot)
% xlabel('t');
% ylabel('$\dot{\theta}$', 'Interpreter', 'latex');
% title('$\dot{\theta}$ vs t', 'Interpreter', 'latex');
% grid on;
% 
% 
aP1 = (V_P/(t_step*9.81)).*diff(y1(: , 5));
aP2 = (V_P/(t_step*9.81)).*diff(y2(: , 5));
% length(aP1)
% length(aP2)
figure;
a1 = plot(t1(1:end-1), abs(aP1), 'c-', 'LineWidth', 1.5);
hold on;
a2 = plot(t2(1:end-1), abs(aP2), 'b--.', 'LineWidth', 1.5);
legend([a1, a2], 'Without Look-Angle Constraint', 'With Look-Angle Constraint', 'Location', 'best')
xlabel('Time [s]');
ylabel('||a_p|| [g]');
title("UAV's lateral acceleration vs Time");
grid on;

fig1 = figure;
mu1 = (180/pi).*(y1(:, 5) - y1(:, 2));
mu2 = (180/pi).*(y2(:, 5) - y2(:, 2));
l1 = plot(t1, mu1, 'c-', 'LineWidth', 1.5);
hold on;
l2 = plot(t2, mu2, 'b--', 'LineWidth', 1.5);
legend([l1, l2], 'Without Look-Angle Constraint', 'With Look-Angle Constraint', 'Location', 'best')
xlabel('Time [s]');
ylabel('Look-Angle (\mu = \alpha_P - \theta) [deg]');
title('Look-Angle vs Time');
grid on;

% % % Plot B vs Time
% % figure;
% % b1 = plot(t1, (1/V_P).*B_1, 'c -', 'LineWidth', 1.5);
% % hold on;
% % b2 = plot(t2, (1/V_P).*B_2, 'b --', 'Linewidth', 1.5);
% % legend([b1, b2], 'Without Look-Angle Constraint', 'With Look-Angle Constraint', 'Location', 'best')
% % xlabel('Time [s]');
% % ylabel('Normalized Bias (B/V_P)');
% % title('B vs Time');
% % grid on;


 
% Extract the trajectories
x_T1 = y1(:, 7); 
y_T1 = y1(:, 8); 
x_P1 = y1(:, 9); 
y_P1 = y1(:, 10); 

x_T2 = y2(:, 7); 
y_T2 = y2(:, 8); 
x_P2 = y2(:, 9); 
y_P2 = y2(:, 10); 

%Plotting the initial positions

% % Animation loop
% for i = 1:length(t)
%     % Update target position
%     set(hT, 'XData', x_T(i), 'YData', y_T(i));
%     % Update pursuer position
%     set(hP, 'XData', x_P(i), 'YData', y_P(i));
%     % Update trajectories
%     set(hTrajT, 'XData', x_T(1:i), 'YData', y_T(1:i));
%     set(hTrajP, 'XData', x_P(1:i), 'YData', y_P(1:i));
%     
%     % Pause to control animation speed
%     pause(0.01);
% end

%Extract the final positions and trajectories
x_T_final1 = x_T1(end); % Final x-position of the target
y_T_final1 = y_T1(end); % Final y-position of the target
x_P_final1 = x_P1(end); % Final x-position of the pursuer
y_P_final1 = y_P1(end); % Final y-position of the pursuer

x_T_final2 = x_T2(end); % Final x-position of the target
y_T_final2 = y_T2(end); % Final y-position of the target
x_P_final2 = x_P2(end); % Final x-position of the pursuer
y_P_final2 = y_P2(end); % Final y-position of the pursuer

% Plot the initial and final positions
fig2 = figure;
% Target's initial position (stationary target)
h1 = plot(x_T1(1), y_T1(1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); 
hold on;
% Pursuer's initial position
h2 = plot(x_P1(1), y_P1(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); 
% Target's final position (stationary)
h3 = plot(x_T_final1, y_T_final1, 'b*', 'MarkerSize', 8); 
% Pursuer's final position
h4 = plot(x_P_final1, y_P_final1, 'r*', 'MarkerSize', 8); 

% Plot the trajectories
h5 = plot(x_T1, y_T1, 'r-', 'LineWidth', 1.5); % Target trajectory (should be stationary)
h6 = plot(x_P1, y_P1, 'c-.', 'LineWidth', 1.5); % Pursuer trajectory
h7 = plot(x_T2, y_T2, 'g-.', 'LineWidth', 1.5); % Target trajectory (should be stationary)
h8 = plot(x_P2, y_P2, 'b-', 'LineWidth', 1.5); % Pursuer trajectory, with look angle constraint

legend([h1, h2, h4, h6, h8], 'Stationary Target', "UAV's Start Position", "UAV's Final Position", 'Without Look-Angle Constraint',  'With Look-Angle Constraint', 'Location', 'best');
xlabel('Downrange (m)');
ylabel('Crossrange (m)');
title("UAV's trajectory for \alpha^d_{P_{f}} = 5\pi/6");
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
