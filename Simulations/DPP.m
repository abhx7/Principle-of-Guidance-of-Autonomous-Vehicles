function dydt = DPP(t, y, VP, VT, delta, alpha_T0, cT)
    % Extract state variables
    R = y(1); % Range
    theta = y(2); % Line-of-sight angle (theta)
    V_theta = y(3); % Angular velocity (V_theta)
    V_R = y(4); % Radial velocity (V_R)
    
    % Target heading angle (alpha_T)
    alpha_T = alpha_T0 + cT * theta; % Maneuvering target
    % cT=0 for Constant target heading
    
    
    % Radial velocity and angular velocity equations for deviated pure pursuit
    V_R = VT * cos(alpha_T - theta) - VP * cos(delta);
    V_theta = VT * sin(alpha_T - theta) - VP * sin(delta);
    
    % Time derivatives
    dR_dt = V_R; % Range rate
    dtheta_dt = V_theta / R; % LOS angle rate
    dV_R_dt =  -dtheta_dt * (V_theta + VP * sin(delta))*(cT-1); 
    dV_theta_dt = dtheta_dt * (V_R + VP * cos(delta))*(cT-1); 
    
    % Return the derivatives
    dydt = [dR_dt; dtheta_dt; dV_R_dt; dV_theta_dt];
end
