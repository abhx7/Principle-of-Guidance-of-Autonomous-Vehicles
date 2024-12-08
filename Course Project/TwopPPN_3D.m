function dydt = TwopPPN_3D(t, y, V_P, V_T, a_nT, alpha_P_d_f, alpha_P0, theta0)
    % State variables (in 3D)
    R = y(1);                % Range between pursuer and target
    theta = y(2);            % LOS angle
    phi = y(3);              % LOS elevation angle
    V_theta = y(4);          % Angular velocity (rate of change of LOS angle in azimuth)
    V_R = y(6);              % Radial velocity (rate of change of range)
    alpha_P_az = y(7);       % Pursuer heading azimuth angle
    alpha_P_el = y(8);       % Pursuer heading elevation angle
    alpha_T_az = y(9);       % Target heading azimuth angle
    alpha_T_el = y(10);      % Target heading elevation angle

    % Constants and constraints
    mu_max = deg2rad(45);    % Maximum look angle constraint in radians
    e = 1e-3;                % Threshold to prevent division by zero

    % Compute look angle in 3D
    mu_az = alpha_P_az - theta;
    mu_el = alpha_P_el - phi;

    % Compute navigation ratio (N) based on constraints
    if abs(mu_az) >= mu_max || abs(mu_el) >= mu_max
        N = 1; % Enforce max look angle constraint
    else
        N = (alpha_P_d_f - alpha_P_az) / (alpha_P_d_f - theta);
        N = min(max(N, 0), 2); % Ensure N is within [0, 2]
    end

    % LOS rates in 3D
    V_theta = V_T * sin(alpha_T_az - theta) - V_P * sin(alpha_P_az - theta);
    V_R = V_T * cos(alpha_T_az - theta) - V_P * cos(alpha_P_az - theta);

    % Rate of change of LOS angles in 3D
    dtheta_dt = V_theta / max(R, e); % Avoid division by zero

    % Lateral acceleration of pursuer due to PPN in 3D
    a_P_az = N * V_P * dtheta_dt;

    % Update target's heading angle rates based on its lateral acceleration
    dalpha_T_az_dt = a_nT / max(V_T, e); % Assuming a_nT is the lateral acceleration
    dalpha_T_el_dt = 0; % Stationary target, no elevation change

    % Update pursuer's heading angle rates in 3D
    dalpha_P_az_dt = a_P_az / max(V_P, e);

    % Compute rate of change of state variables
    dR_dt = V_R;
    dV_theta_dt = V_T * cos(alpha_T_az - theta) * (dalpha_T_az_dt - dtheta_dt) - V_P * cos(alpha_P_az - theta) * (dalpha_P_az_dt - dtheta_dt);
    dV_R_dt = -V_T * sin(alpha_T_az - theta) * (dalpha_T_az_dt - dtheta_dt) + V_P * sin(alpha_P_az - theta) * (dalpha_P_az_dt - dtheta_dt);

    % Construct the derivative vector (R', theta', phi', V_theta', V_R', alpha_P_az', alpha_P_el', alpha_T_az', alpha_T_el')
    dydt = [dR_dt; dtheta_dt; 0; dV_theta_dt; 0; dV_R_dt; dalpha_P_az_dt; 0; dalpha_T_az_dt; dalpha_T_el_dt];
end
