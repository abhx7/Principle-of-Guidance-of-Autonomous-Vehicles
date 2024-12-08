function dydt = equations_TwopPPN_without_mu(t, y,  params)
    V_P = params(1);
    V_T = params(2);
    N = params(4);

    e = 1e-3; %threshold to prevent division by zero, etc

    % State variables
    R = y(1);         % Range between pursuer and target
    theta = y(2);     % LOS angle
    V_theta = y(3);   % Angular velocity (rate of change of LOS angle)
    V_R = y(4);       % Radial velocity (rate of change of range)
    alpha_P = y(5);   % Pursuer heading angle
    alpha_T = y(6);   % Target heading angle

    V_theta = V_T * sin(alpha_T - theta) - V_P * sin(alpha_P - theta);
    V_R = V_T * cos(alpha_T - theta) - V_P * cos(alpha_P - theta);

    % Rate of change of LOS angle
    dtheta_dt = V_theta / R;%max(R,e);
    dR_dt = V_R;

    % Lateral acceleration of pursuer due to PPN
    a_P = N * V_P * dtheta_dt;

    % Update target's heading angle based on its lateral acceleration
    dalpha_T_dt = 0;  % Change in target's heading angle due to lateral acceleration
    % Update heading of the pursuer
    dalpha_P_dt = a_P / max(V_P, e);

    % Compute rate of change of state variables
    dV_theta_dt = V_T * cos(alpha_T - theta)*(dalpha_T_dt-dtheta_dt) - V_P * cos(alpha_P - theta)*(dalpha_P_dt-dtheta_dt);
    dV_R_dt = -V_T * sin(alpha_T - theta)*(dalpha_T_dt-dtheta_dt) + V_P * sin(alpha_P - theta)*(dalpha_P_dt-dtheta_dt);

    % Construct the derivative vector (R', theta', V_theta', V_R', alpha_P', alpha_T')
    dydt = [dR_dt; dtheta_dt; dV_theta_dt; dV_R_dt; dalpha_P_dt; dalpha_T_dt];
end