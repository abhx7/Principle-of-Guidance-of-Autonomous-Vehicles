% Clear workspace and close all figures
clear;
clc;
close all;

% Define Simulation Parameters
V_T = 0; % Target speed in m/s
V_P = 50; % Pursuer speed in m/s (modify as needed)
nu = V_P/V_T; 

alpha_P_df = deg2rad(-50);%-pi/4;
disp(['Desired alpha_P_df = ', num2str(rad2deg( alpha_P_df))])
e = 1; % threshold to prevent division by zero

delta0 = deg2rad(45);% atan(400/500); % initial Deviation angle in radians
theta0 = deg2rad(0); % Initial LOS angle in radians
alpha_P0 = theta0 + delta0; % Initial pursuer angle in radians
alpha_T0 = 0;
disp(['alpha_P0 = ', num2str(rad2deg(alpha_P0))])
R0 = 2500; % Initial separation distance in meters

% Calculate initial velocities
V_R0 = V_T * cos(alpha_T0 - theta0) - V_P * cos(delta0);
V_theta0 = V_T * sin(alpha_T0 - theta0) - V_P * sin(delta0);

% Time vector
t_step = 0.1;
t_end = 500; % End time for simulation (adjust if needed)
t_span = 0:t_step:t_end;

% Initial conditions 
y0 = [R0, theta0, V_theta0, V_R0, alpha_P0, 0];

N0 = (alpha_P_df - alpha_P0)/(alpha_P_df - theta0);

N_ori = linspace(-2,N0,5);
scenario_count = length(N_ori); 
  

% Define terminal condition (example: stop when R reaches a certain threshold)
threshold_R = 0.01*R0;  % Example threshold
terminal_condition = @(y) y(1) < threshold_R;

global N_values t_ori; 
aP_matrix = zeros(length(t_span),scenario_count);

N_final = [2, 2.5, 3, 3.5, 4];
%N_final = [2];

% Define a set of colors (e.g., using RGB triplets)
colors = lines(length(N_final)); % 'lines' colormap generates a set of distinct colors

for i = 1:length(N_final)
    N_f = N_final(i);
    % initialise cost variable
    cost = zeros(1, scenario_count);  
    totaltime = zeros(1, scenario_count); 

    for scenario_num = 1:scenario_count
        N_values = [];
        t_ori = 0;
    
        % Set up parameters
        params = [V_P, V_T, alpha_P_df, N_ori(scenario_num), N_f];
        
        % Call the solver
        y = rk4_solver(@equations_TwopPPN, @TwopPPN_without_mu, y0, t_span, t_step,...
            params, terminal_condition);
        %y = rk4_solver(@equations_TwopPPN, @TwopPPN_with_mu, y0, t_span, t_step,...
        %    params, terminal_condition);
    
        if length(N_values) < length(t_span)
            N_values = [N_values; zeros(length(t_span) - length(N_values), 1)];
        end
    
        N_matrix(:, scenario_num) = N_values;
        
        R = y(:, 1); % Range
        theta = y(:, 2); % LOS angle
        V_theta = y(:, 3); % Angular velocity
        V_R = y(:, 4); % Radial velocity
        alpha_P = y(:, 5); % Pursuer heading angle
        alpha_T = y(:, 6); % Target heading angle
    
    % %     t_ori
    % %     length(R)
    
        % Define the acceleration arrays over their respective ranges
        t_ori_idx = 1:ceil(t_ori/t_step);  % Use proper indexing range
        t_end_idx = ceil(t_ori/t_step)+1:length(R);  % Full span till end
        
        dtheta_dt_ori = diff(theta(t_ori_idx))/t_step;
        dtheta_dt_2 = diff(theta(t_end_idx))/t_step;
        dtheta_dt_ori=[dtheta_dt_ori' dtheta_dt_2(1)]';
        dtheta_dt_2=[dtheta_dt_ori(1) dtheta_dt_2']';
        % Calculate accelerations
%         aP_ori = N_matrix(t_ori_idx, scenario_num) .* V_P .* V_theta(t_ori_idx) ./ R(t_ori_idx);
%         aP_2 = N_matrix(t_end_idx, scenario_num) .* V_P .* V_theta(t_end_idx) ./ R(t_end_idx);
        aP_ori = N_matrix(t_ori_idx, scenario_num) .* V_P .*dtheta_dt_ori;
        aP_2 = N_matrix(t_end_idx, scenario_num) .* V_P .* dtheta_dt_2;
    %     aP_ori = N_values(t_ori_idx) .* V_P .* Vtheta(t_ori_idx) ./ R(t_ori_idx);
    %     aP_2 = N_values(t_end_idx) .* V_P .* Vtheta(t_end_idx) ./ R(t_end_idx);
    
    
        % Accumulate cost as the sum of squared accelerations over time
        cost(scenario_num) = (sum(aP_ori.^2) + sum(aP_2.^2)) * t_step;
        %cost(scenario_num) = (sum(abs(aP_ori)) + sum(abs(aP_2))) * t_step;
    
        totaltime(scenario_num) = t_span(length(R));
    
        % Combined plot: R, theta, V_theta, V_R
%         figure;
%         subplot(2, 2, 1); plot(t_span(1:length(R)), R); xlabel('Time (s)'); ylabel('Range (m)'); title('Range R');
%         subplot(2, 2, 2); plot(t_span(1:length(R)), theta); xlabel('Time (s)'); ylabel('\theta (rad)'); title('LOS Angle \theta');
%         subplot(2, 2, 3); plot(t_span(1:length(R)), V_theta); xlabel('Time (s)'); ylabel('V_\theta (rad/s)'); title('Angular Velocity V_\theta');
%         subplot(2, 2, 4); plot(t_span(1:length(R)), V_R); xlabel('Time (s)'); ylabel('V_R (m/s)'); title('Radial Velocity V_R');
%         sgtitle(['Combined Plots', 'N_f = ', num2str(N_f), 'N_f = ', num2str(N_f)]);
      
%         % Plot aP
%         figure;
%         plot(t_span(1:length(R)),[aP_ori',aP_2']);
%         xlabel('t');
%         ylabel('aP');
%         title('Lateral Accelaration');
%         grid on;

        % Plot aP
        figure;
        
        % First subplot for the combined plot of aP_ori and aP_2
        subplot(3, 1, 1);
        plot(t_span(1:length(R)), [aP_ori', aP_2']);
        xlabel('t');
        ylabel('aP');
        title('Lateral Acceleration');
        grid on;
        
        % Second subplot for aP_ori during the orientation phase
        subplot(3, 1, 2);
        plot(t_span(t_ori_idx), aP_ori');
        xlabel('t');
        ylabel('aP_{ori}');
        title(['Lateral Acceleration Orientation Phase, N = ', num2str(N_ori(scenario_num))]);
        grid on;
        
        % Third subplot for aP_2 during the second phase, adjusted to start where aP_ori ends
        subplot(3, 1, 3);
        t_aP_2 = t_span(t_end_idx); % Adjust time vector to start where aP_ori ends
        plot(t_aP_2, aP_2');
        xlabel('t');
        ylabel('aP_{2}');
        title(['Lateral Acceleration Phase 2, N = ', num2str(N_f)]);
        grid on;

%         % Plot look angle  
%         mu = rad2deg(alpha_P - theta);
%         figure;
%         plot(t_span(1:length(R)), mu); xlabel('Time (s)'); ylabel('Look Angle mu (deg)'); title('Look Angle');
%         grid on;
    
        plot_real_time_chase(t_span(1:length(R)), R, theta, V_T, V_P, 0, alpha_P,...
            scenario_num, colors, i, scenario_count);
    
    
        aP_matrix(t_ori_idx, scenario_num) = aP_ori;
        aP_matrix(t_end_idx, scenario_num) = aP_2;
    
        rad2deg(alpha_P(end))
    end

    
    % Plot cost vs theta_d
    figure(1001);
    hold on 
    plot(N_ori, cost, '-o', 'LineWidth', 1.5, 'DisplayName', ['N_f = ', num2str(N_f)]);
    xlabel('N');
    ylabel('Cost');
    title('Cost vs N');
    grid on;
    legend('show');


    % Plot time
    figure(1002);
    hold on 
    plot(N_ori, totaltime, 'LineWidth', 1.5, 'DisplayName', ['N_f = ', num2str(N_f)]);
    xlabel('N');
    ylabel('Time');
    title('Total Time');
    grid on;
    legend('show');
end



%--- Function for Real-Time Plotting of the Chase ---%
function plot_real_time_chase(t, R, theta, V_T, V_P, alpha_T, alpha_P, scenario_num, colors, N_f_i, len_N_ori)
    % Initialize positions of the pursuer and target
    pursuer_pos = [0, 0]; % Initial pursuer position at origin
    target_pos = [R(1) * cos(theta(1)), R(1) * sin(theta(1))]; % Initial target position

    % Arrays to store trajectory history
    pursuer_traj = pursuer_pos;
    target_traj = target_pos;
   
    % Create figure for real-time plotting
    figure(1000);
    hold on;
    grid on;
    axis equal;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Real-Time Pursuer and Target Trajectory');


%     % Initialize plot handles for pursuer and target
%     hPursuer = plot(pursuer_pos(1), pursuer_pos(2), 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Pursuer');
%     hTarget = plot(target_pos(1), target_pos(2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Target');
%     hTargetTraj = plot(target_traj(:,1), target_traj(:,2), 'r-', 'DisplayName', 'Target Trajectory');
    if scenario_num == len_N_ori
        hPursuerTraj = plot(pursuer_traj(:,1), pursuer_traj(:,2), 'b--',...
         'DisplayName', num2str(N_f_i), 'Tag', num2str(N_f_i),...
         'Color', colors(mod(N_f_i-1, size(colors, 1)) + 1, :));
    else
        hPursuerTraj = plot(pursuer_traj(:,1), pursuer_traj(:,2), 'b--',...
         'DisplayName', '' , 'Tag', num2str(N_f_i),...
         'Color', colors(mod(N_f_i-1, size(colors, 1)) + 1, :));
    end 
    legend('show');
    % Loop through time steps for real-time updates
    for i = 2:length(t)
        % Update target position based on constant velocity
        dt = t(i) - t(i-1); % Time step
        target_pos = target_pos + [V_T * cos(alpha_T),  V_T * sin(alpha_T)] * dt;

        % Update pursuer position based on the velocity and time step
        %pursuer_pos = pursuer_pos + [V_P * cos(alpha_P(i)), V_P * sin(alpha_P(i))] * dt;
        pursuer_pos = target_pos - [R(i) * cos(theta(i)),  R(i) * sin(theta(i))];

        % Append current positions to trajectory history
        pursuer_traj = [pursuer_traj; pursuer_pos];
        target_traj = [target_traj; target_pos];

        % Ensure plot handles are valid before updating
        if isvalid(hPursuerTraj) %isvalid(hPursuer) && isvalid(hTarget) && isvalid(hPursuerTraj) && isvalid(hTargetTraj)
            % Update plot positions and trajectories
%             set(hPursuer, 'XData', pursuer_pos(1), 'YData', pursuer_pos(2));
%             set(hTarget, 'XData', target_pos(1), 'YData', target_pos(2));
             set(hPursuerTraj, 'XData', pursuer_traj(:,1), 'YData', pursuer_traj(:,2));
%             set(hTargetTraj, 'XData', target_traj(:,1), 'YData', target_traj(:,2));

            % Pause to create real-time effect
            %pause(1e-6);
        else
            % Exit loop if plot handles are no longer valid (e.g., figure closed)
            break;
        end
    end
end
