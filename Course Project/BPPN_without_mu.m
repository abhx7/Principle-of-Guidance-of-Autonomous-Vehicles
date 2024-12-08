function dy_dt = BPPN_without_mu(t, y, V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0)
    %N = 3;
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
   
    % Initialize persistent variables for storing the values
    persistent prev_vtheta_by_r t_2 theta_2
    
    prev_vtheta_by_r = V_theta_0/R_0;
    
    % Current value of V_theta/R
    current_vtheta_by_r = V_theta / R;
    
    % Check if sign change occurs and the values are not yet stored
    tolerance = 0.01;  % Define a small tolerance
    if current_vtheta_by_r > 0 && isempty(t_2) && abs(current_vtheta_by_r) < tolerance && sign(current_vtheta_by_r) ~= sign(prev_vtheta_by_r(end))
        t_2 = t;         % Store the time when sign changes
        theta_2 = theta;  % Store the corresponding theta value
        alpha_P_2 = alpha_P;
        fprintf('V_theta/R changes sign at t2 = %.4f seconds, theta2 = %.4f rad\n', t_2, theta_2);
    end
    
    % Update the previous value
    prev_vtheta_by_r = current_vtheta_by_r;


    % phase - 1
   
    B = V_P*abs(alpha_P)/5;
    %B = 0.02*V_P;
    N = 2;
    aP = N * V_P * V_theta/R + B*sign(V_theta_0/R_0);

    % phase - 2
    %if ~isempty(t_2) && ~isempty(theta_2) && all(theta >= theta_2) && all(t > t_2)
    if t >= t_2
        B = V_P*abs(alpha_P)/10;
        N = -2;
        %N = 0.1;
        aP = N * V_P * V_theta/R + B*sign(V_theta_0/R_0);
    end

   % phase - 3

    theta_3 = -0.175;

    % Initialize persistent variable for storing the time when theta_3 is reached
    persistent t_3 alpha_P_3

    % Check if t_2 is set, theta has reached or exceeded theta_3, and t_3 has not been set
    if isempty(t_3)  && ~isempty(t_2) && t > t_2(end) && theta >= theta_3
        t_3 = t;  % Store the current time as t_3 when theta reaches theta_3 and t > t_2
        alpha_P_3 = alpha_P;
        fprintf('Theta reaches theta_3 = %.4f radians at t_3 = %.4f seconds, after t_2 = %.4f\n', theta, t_3, t_2);
    end

    % Proceed with Phase 3 logic after theta_3 is reached and t > t_3
    %if ~isempty(t_3) && t > t_3(end)
    if t >= t_3
        B = 0;
        N = (alpha_P_df - alpha_P) / (alpha_P_df - theta);
        if N < 2
            N = (2/pi) * abs(alpha_P_3 - theta_3);
            aP = N * V_P * V_theta / R;
        end
        aP = N * V_P * V_theta / R;
    end


    dy_dt = zeros(10, 1);

    dy_dt(1) = V_T*cos(alpha_T - theta) - V_P*cos(mu);  
    dy_dt(2) =  (V_T*sin(alpha_T - theta) - V_P*sin(mu))/R;

%     if aP > 5
%         aP = 5;
%     elseif aP < -5
%         aP = -5;
%     end
   
    dy_dt(5) = aP/V_P;                          
    dy_dt(6) = 0;  
%     k = N - 1;
%     phi_0 = alpha_P - N * theta;

%     dy_dt(4) = (V_theta + V_P * N * sin(k*theta + phi_0)) * V_theta/R;
%     dy_dt(3) = (V_R + V_P * N * cos(k * theta + phi_0))* (-V_theta/R);

    dy_dt(4) = -V_T*sin(alpha_T - theta)*(dy_dt(6) - V_theta/R) + V_P*sin(alpha_P - theta)*(dy_dt(5) - V_theta/R);      
    dy_dt(3) = V_T*cos(alpha_T - theta)*(dy_dt(6) - V_theta/R) - V_P*cos(alpha_P - theta)*(dy_dt(5) - V_theta/R);  
    %dy_dt(4) = (V_theta/R) * (V_theta + N * V_P* sin(alpha_P - theta));
    %dy_dt(3) = -(V_theta/R) * (V_R + N * V_P * cos(alpha_P - theta));
   
    dy_dt(7) = V_T*cos(alpha_T);                
    dy_dt(8) = V_T*sin(alpha_T);                
    dy_dt(9) = V_P*cos(alpha_P);                
    dy_dt(10) = V_P*sin(alpha_P);                 
end
