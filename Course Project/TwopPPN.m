function dydt = TwopPPN(t, y,  V_P, V_T, a_nT, alpha_P_d_f, alpha_P0, theta0)
    % State variables
    R = y(1);         % Range between pursuer and target
    theta = y(2);     % LOS angle
    V_theta = y(3);   % Angular velocity (rate of change of LOS angle)
    V_R = y(4);       % Radial velocity (rate of change of range)
    alpha_P = y(5);   % Pursuer heading angle
    %V_P = y(6);       % Pursuer speed
    alpha_T = y(6);   % Target heading angle
    %V_T = y(8);       % Target speed

    %%Look Angle Constraint
    mu_max = deg2rad(45);

    e = 1e-3; %threshold to prevent division by zero, etc

    mu = alpha_P - theta;
    if (alpha_P_d_f - alpha_P)/(alpha_P_d_f - theta) < 2
        N = (2/pi)*abs(alpha_P0-theta0);
%         if abs(mu) >= mu_max
%             N=1;
%         end  
    else
        N = (alpha_P_d_f - alpha_P)/(alpha_P_d_f - theta);
    end  
   

    V_theta = V_T * sin(alpha_T - theta) - V_P * sin(alpha_P - theta);
    V_R = V_T * cos(alpha_T - theta) - V_P * cos(alpha_P - theta);

    % Rate of change of LOS angle
    dtheta_dt = V_theta / R;%max(R,e);

    % Lateral acceleration of pursuer due to PPN
    a_P = N * V_P * dtheta_dt;

%     %Cap the maximum latax control input
%     if a_P > 10
%         a_P = 10;
%     elseif a_P < -10
%        a_P = -10;
%     end

    % Update target's heading angle based on its lateral acceleration
    dalpha_T_dt = a_nT / max(V_T, e);  % Change in target's heading angle due to lateral acceleration

    % Update heading of the pursuer
    dalpha_P_dt = a_P / max(V_P, e);
    %dVP_dt = V_P * sin(alpha_P - theta) * (dalpha_P_dt - dtheta_dt);

    % Compute rate of change of state variables
    
    dR_dt = V_R;
    dV_theta_dt = V_T * cos(alpha_T - theta)*(dalpha_T_dt-dtheta_dt) - V_P * cos(alpha_P - theta)*(dalpha_P_dt-dtheta_dt);
    dV_R_dt = -V_T * sin(alpha_T - theta)*(dalpha_T_dt-dtheta_dt) + V_P * sin(alpha_P - theta)*(dalpha_P_dt-dtheta_dt);

    % Construct the derivative vector (R', theta', V_theta', V_R', alpha_P', alpha_T')
    %dydt = [dR_dt; dtheta_dt; dV_theta_dt; dV_R_dt; dalpha_P_dt; dVP_dt ; dalpha_T_dt; 0];
    dydt = [dR_dt; dtheta_dt; dV_theta_dt; dV_R_dt; dalpha_P_dt; dalpha_T_dt];
end

% dalpha_T_dt = a_T * cos(alpha_T - theta);   % Target heading angle rate
% dVT_dt = a_T * sin(alpha_T - theta);        % Target velocity rate