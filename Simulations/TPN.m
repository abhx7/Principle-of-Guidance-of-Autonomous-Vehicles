function dydt = TPN(t, y, c, b, Nd, RTPN)
    % Extract state variables
    R = y(1);           % Range
    theta = y(2);        % Line-of-sight angle (theta)
    V_theta = y(3);      % Angular velocity (V_theta)
    V_R = y(4);          % Radial velocity (V_R)
    alpha_P = y(5);      % Pursuer heading angle (alpha_P)
    V_P = y(6);          % Pursuer speed (V_P, allows for dynamic updates)
    alpha_T = y(7);      % Target heading angle (alpha_T)
    V_T = y(8);          % Target speed (VT, allows for dynamic updates)

    % Target lateral acceleration a_T based on maneuvering law a_T = b / V_theta
    if V_theta ~= 0
        a_T = b / V_theta;  % Maneuvering target acceleration
    else
        a_T = b / 1e-3;  % Prevent division by zero
    end

    % Compute LOS rate and relative velocity components
    V_R = V_T * cos(alpha_T - theta) - V_P * cos(alpha_P - theta);
    V_theta = V_T * sin(alpha_T - theta) - V_P * sin(alpha_P - theta);
    
    if RTPN == 1
        c = - Nd *V_R; %RTPN
    end

    % PN law: a_P = N_c * |dtheta/dt|
    if R ~= 0
        a_P = c * abs(V_theta / R);  % Pursuer's lateral acceleration using PN
    else
        a_P = c * abs(V_theta /1e-3);  % Prevent division by zero
    end
    

    % Time derivatives
    dR_dt = V_R;                                % Range rate
    dtheta_dt = V_theta / R;                    % LOS angle rate
    dV_R_dt = dtheta_dt * V_theta;              % Radial velocity rate
    dV_theta_dt = -dtheta_dt * V_R + a_T - a_P; % Angular velocity rate
    dalpha_P_dt = a_P * cos(alpha_P - theta);   % Pursuer heading angle rate
    dVP_dt = a_P * sin(alpha_P - theta);        % Pursuer's velocity rate
    % Target dynamics
    dalpha_T_dt = a_T * cos(alpha_T - theta);   % Target heading angle rate
    dVT_dt = a_T * sin(alpha_T - theta);        % Target velocity rate

    % Return derivatives [dR/dt, dtheta/dt, dV_R/dt, dV_theta/dt, dalpha_P/dt, dVP/dt, dalpha_T/dt, dVT/dt]
    dydt = [dR_dt; dtheta_dt; dV_R_dt; dV_theta_dt; dalpha_P_dt; dVP_dt; dalpha_T_dt; dVT_dt];
end
