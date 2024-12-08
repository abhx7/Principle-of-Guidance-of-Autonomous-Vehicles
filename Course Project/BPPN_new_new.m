function [dy_dt, B] = BPPN_new_new(t, y, V_P, V_T, alpha_P_df, alpha_P0, theta_0, V_theta_0, R_0)
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
    int_B = y(11);
    dy_dt = zeros(11, 1);

    mu_th = pi/4;
    mu = alpha_P - theta;
   
    % Initialize persistent variables for storing the values
    persistent prev_vtheta_by_r t_2 theta_2 t_3 alpha_P_3 theta_3 t_cap t_prev intt_B
    
    prev_vtheta_by_r = V_theta_0/R_0;
    
    % Current value of V_theta/R
    current_vtheta_by_r = V_theta / R;

    % Check if sign change occurs and the values are not yet stored
    tolerance = 0.001;  % Define a small tolerance
    if current_vtheta_by_r > 0 && isempty(t_2) && abs(current_vtheta_by_r) < tolerance && sign(current_vtheta_by_r) ~= sign(prev_vtheta_by_r)
        t_2 = t;         % Store the time when sign changes
        theta_2 = theta;  % Store the corresponding theta value
        alpha_P_2 = alpha_P;
        fprintf('V_theta/R changes sign at t2 = %.4f seconds, theta2 = %.4f rads\n', t_2, theta_2);
        B = V_P*abs(alpha_P)/5;
        intt_B = 0;
        t_prev = t_2;
        dy_dt(11) = B;
        t;
    end
    
    % Update the previous value
    prev_vtheta_by_r = current_vtheta_by_r;

    %% phase - 1
    B = V_P*abs(alpha_P)/5;
    if abs(mu) >= mu_th
        N = 1;    
    else 
        N = 2;
    end
    aP = N * V_P * V_theta/R + B*sign(V_theta_0/R_0);   

%     if isempty(t_2)
%         int_B;
%         t;
%     end

    %% phase - 2
    if ~isempty(t_2) && t > t_2 %&& isempty(t_cap)
        if  t > t_prev 
           if isempty(t_3) || t < t_3 
            B = V_P*abs(alpha_P)/10;
            intt_B = intt_B + B*(t-t_prev);
            (N-1)*V_P*(theta - theta_2);
            dy_dt(11) = B;
            if theta < alpha_P_df && abs(intt_B) < (N-1)*V_P*(theta - theta_2) + V_P*mu_th && abs(intt_B) > (N-1)*V_P*(theta - theta_2) %&& isempty(t_cap)
                %disp('hi');
                if abs(mu) >= mu_th
                    N = 1;
                    B = 0;
                else 
                    N = -2;
                end
                %aP = N*V_P*V_theta/R + B*sign(V_theta_0/R_0);
            elseif isempty(t_3) 
                %disp('hey');
                if isempty(t_cap)
                    t_cap = t;
                    theta_cap = theta
                    alpha_P_cap = alpha_P;
                end
                0.8*theta_cap
                if alpha_P_df > 0.5*theta_cap && alpha_P_df < pi + theta_cap
                    t_3 = t_cap;
                    theta_3 = theta_cap;
                    alpha_P_3 = alpha_P_cap;
                    fprintf('Theta reaches theta_3 = %.4f radians at t_3 = %.4f seconds, after t_2 = %.4f\n', theta, t_3, t_2);
                else 
                    N = 1;
                    B = 0;
                    if alpha_P_df > 0.8*(theta) && alpha_P_df < 0.8*(pi + theta)
                        t_3 = t;
                        theta_3 = theta;
                        alpha_P_3 = alpha_P;
                        fprintf('Theta reaches theta_3 = %.4f radians at t_3 = %.4f seconds, after t_2 = %.4f\n', theta, t_3, t_2);
                    end
                end
                if abs(mu) >= mu_th
                    N = 1;
                    B = 0;
                else 
                    N = -2;
                end
            end
            aP = N * V_P * V_theta/R + B*sign(V_theta_0/R_0);
           else
               dy_dt(11) = 0;
           end
        else
            dy_dt(11) = 0;
 
        end
       t_prev = t;
    end

   %% phase - 3
    theta_3;
    t_2;
    t_3;

    % Proceed with Phase 3 logic after theta_3 is reached and t > t_3
    if ~isempty(t_3) && t > t_3
        B = 0;
        N = (alpha_P_df - alpha_P) / (alpha_P_df - theta);
        if N < 2
            if abs(mu) >= mu_th
                N = 1;
            else
                N = (2/pi) * abs(alpha_P_3 - theta_3);
            end
        end
        aP = N * V_P * V_theta / R;
    end

%     % Check the integral conditions specified in 4b
%     integral_threshold = (N - 1) * V_P * (theta - theta_2);

    dy_dt(1) = V_T*cos(alpha_T - theta) - V_P*cos(mu);  
    dy_dt(2) = (V_T*sin(alpha_T - theta) - V_P*sin(mu))/R;   
    dy_dt(5) = aP/V_P;                          
    dy_dt(6) = 0;  
    dy_dt(4) = -V_T*sin(alpha_T - theta)*(dy_dt(6) - V_theta/R) + V_P*sin(alpha_P - theta)*(dy_dt(5) - V_theta/R);      
    dy_dt(3) = V_T*cos(alpha_T - theta)*(dy_dt(6) - V_theta/R) - V_P*cos(alpha_P - theta)*(dy_dt(5) - V_theta/R);  
    dy_dt(7) = V_T*cos(alpha_T);                
    dy_dt(8) = V_T*sin(alpha_T);                
    dy_dt(9) = V_P*cos(alpha_P);                
    dy_dt(10) = V_P*sin(alpha_P);  
end
