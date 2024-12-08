function dydt = BPPN(t, y, V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0)
    % State variables
    R = y(1);         % Range between pursuer and target
    theta = y(2);     % LOS angle
    V_theta = y(3);   % Angular velocity (rate of change of LOS angle)
    V_R = y(4);       % Radial velocity (rate of change of range)
    alpha_P = y(5);   % Pursuer heading angle
    %V_P = y(6);       % Pursuer speed
    alpha_T = y(6);   % Target heading angle
    %V_T = y(8);       % Target speed
    e = 1e-2;

    mu_max = pi/4;
    mu = alpha_P - theta;
   
    %% Checking if phase 1 is over
    % Initialize persistent variables for storing the values
    persistent prev_vtheta_by_r t_2 theta_2
    
    prev_vtheta_by_r = V_theta_0/R_0;
    
    % Current value of V_theta/R
    current_vtheta_by_r = V_theta / R;
   
    % Check if sign change occurs and the values are not yet stored
    if isempty(t_2) && abs(current_vtheta_by_r) < e && sign(current_vtheta_by_r) ~= sign(prev_vtheta_by_r(end))
        t_2 = t;         % Store the time when sign changes
        theta_2 = theta; % Store the corresponding theta value
        alpha_P_2 = alpha_P;
        %fprintf('V_theta/R changes sign at t2 = %.4f seconds, theta2 = %.4f degrees, alpha_P_2 = %.4f degrees \n', t_2, rad2deg(theta_2(end)), rad2deg(alpha_P_2));
        disp(['V_theta/R changes sign at t2 = ', num2str(t_2), ' seconds, theta2 = ', num2str(rad2deg(theta_2)), ' degrees, alpha_P_2 = ', num2str(rad2deg(alpha_P_2)), ' degrees']);
    end
    
    
    % Update the previous value
    prev_vtheta_by_r = current_vtheta_by_r;

    %% Guidance for each Phase

    %% Phase - 1
    B = V_P*0.02;
    N = 2;
    if abs(mu) >= mu_max
          N=1;
    end  
    aP = N * V_P * V_theta/R + B*sign(V_theta_0/R_0);

    %% Phase - 2
    %if ~isempty(t_2) && ~isempty(theta_2) && all(theta >= theta_2) && all(t > t_2)
    if t >= t_2
        %B = V_P*abs(alpha_P)/10;
        N = -2;
        N = 0.1;
        if abs(mu) >= mu_max
          N=1;
        end  
        aP = N * V_P * V_theta/R + B*sign(V_theta_0/R_0);
    end

   %% Phase - 3

    theta_3 = -0.6 ;

    % Initialize persistent variable for storing the time when theta_3 is reached
    persistent t_3 alpha_P_3


    % Check if t_2 is set, theta has reached or exceeded theta_3, and t_3 has not been set
    if isempty(t_3) && ~isempty(t_2) && (t > t_2(end)) && (theta >= theta_3)
        t_3 = t;  % Store the current time as t_3 when theta reaches theta_3 and t > t_2
        alpha_P_3 = alpha_P;
        fprintf('Theta reaches theta_3 = %.4f radians at t_3 = %.4f seconds, after t_2 = %.4f\n', theta, t_3, t_2);
    end

    % Proceed with Phase 3 logic after theta_3 is reached and t > t_3
    %~isempty(t_3) && (t > t_3(end))
    if t >= t_3
        N = (alpha_P_df - alpha_P) / (alpha_P_df - theta);
        if abs(mu) >= mu_max
          N=1;
        end  
        aP = N * V_P * V_theta / R;
        if N < 2
            N = (2/pi) * abs(alpha_P_3 - theta_3);
            if abs(mu) >= mu_max
                 N=1;
            end  
            aP = N * V_P * V_theta / R;
        end
    end

    %% Define all the derivaitves

    dR_dt = V_T*cos(alpha_T - theta) - V_P*cos(mu);  
    dtheta_dt =  (V_T*sin(alpha_T - theta) - V_P*sin(mu))/R;

%     if aP > 5
%         aP = 5;
%     elseif aP < -5
%         aP = -5;
%     end
   
    dalpha_P_dt = aP/max(V_P,e);                          
    dalpha_T_dt = 0;  

    dV_theta_dt = V_T * cos(alpha_T - theta)*(dalpha_T_dt-dtheta_dt) - V_P * cos(alpha_P - theta)*(dalpha_P_dt-dtheta_dt);
    dV_R_dt = -V_T * sin(alpha_T - theta)*(dalpha_T_dt-dtheta_dt) + V_P * sin(alpha_P - theta)*(dalpha_P_dt-dtheta_dt);

    
    dydt = [dR_dt; dtheta_dt; dV_theta_dt; dV_R_dt; dalpha_P_dt; dalpha_T_dt];
end