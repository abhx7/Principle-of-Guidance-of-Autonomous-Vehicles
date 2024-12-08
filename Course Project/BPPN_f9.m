function [dy_dt, B] = BPPN_f9(t, y, V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0, current_B, current_theta_3)
    % Define persistent variables
    persistent prev_vtheta_by_r t_2 theta_2 t_3 alpha_P_3

    prev_vtheta_by_r = V_theta_0/R_0;

    % Check if persistent variables need to be reset
    if t == 0  
        % Reset persistent variables for each bias loop
        prev_vtheta_by_r = [];
        t_2 = [];
        theta_2 = [];
        t_3 = [];
        alpha_P_3 = [];
    end
    
    R = y(1);
    theta = y(2);
    V_theta = y(3);
    V_R = y(4);
    alpha_P = y(5);
    alpha_T = y(6);
    x_T = y(7);
    y_T = y(8);
    x_P = y(9);
    y_P = y(10);

    mu_th = pi/4;
    mu = alpha_P - theta;

   
    current_vtheta_by_r = V_theta / R;

    tolerance = 0.01;
    if current_vtheta_by_r > 0 && isempty(t_2) && abs(current_vtheta_by_r) < tolerance && sign(current_vtheta_by_r) ~= sign(prev_vtheta_by_r(end))
        t_2 = t;
        theta_2 = theta;
        alpha_P_2 = alpha_P;
        fprintf('V_theta/R changes sign at t2 = %.4f seconds, theta2 = %.4f degrees\n', t_2, theta_2*180/pi);
    end

    prev_vtheta_by_r = current_vtheta_by_r;

    % Phase - 1
    B = current_B * V_P;
    if abs(mu) >= mu_th
        N = 1;
    else
        N = 2;
    end
    aP = N * V_P * V_theta/R + B*sign(V_theta_0/R_0);

    % Phase - 2
    if t >= t_2
        %B = current_B * V_P;
        if abs(mu) >= mu_th
            N = 1;
        else
            N = -2;
        end

        aP = N * V_P * V_theta/R + B*sign(V_theta_0/R_0);
    end

    theta_3 = current_theta_3;

    if isempty(t_3) && ~isempty(t_2) && t > t_2(end) && theta >= theta_3
        t_3 = t;
        alpha_P_3 = alpha_P;
        fprintf('Theta reaches theta_3 = %.4f radians at t_3 = %.4f seconds, after t_2 = %.4f\n', theta, t_3, t_2);
    end

    if t >= t_3
        B = 0;
        N = (alpha_P_df - alpha_P) / (alpha_P_df - theta);
        if N < 2
            if abs(mu_th) >= mu_th
                N = 1;
            else
                N = (2/pi) * abs(alpha_P_3 - theta_3);
            end
        end
        aP = N * V_P * V_theta / R;
    end

    dy_dt = zeros(10, 1);
    dy_dt(1) = V_T*cos(alpha_T - theta) - V_P*cos(mu);
    dy_dt(2) =  (V_T*sin(alpha_T - theta) - V_P*sin(mu))/R;
    dy_dt(5) = aP/V_P;
    dy_dt(6) = 0;
    dy_dt(4) = -V_T*sin(alpha_T - theta)*(dy_dt(6) - V_theta/R) + V_P*sin(alpha_P - theta)*(dy_dt(5) - V_theta/R);      
    dy_dt(3) = V_T*cos(alpha_T - theta)*(dy_dt(6) - V_theta/R) - V_P*cos(alpha_P - theta)*(dy_dt(5) - V_theta/R);  
    dy_dt(7) = V_T*cos(alpha_T);                
    dy_dt(8) = V_T*sin(alpha_T);                
    dy_dt(9) = V_P*cos(alpha_P);                
    dy_dt(10) = V_P*sin(alpha_P); 
end
