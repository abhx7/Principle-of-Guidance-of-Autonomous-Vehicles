function vars = TwopPPN_without_mu(t, y, params)
    % Parameters 
    V_P = params(1);
    V_T = params(2);
    alpha_P_df = params(3);
    N_ori = params(4);
    
    % State variables
    R = y(1);         % Range between pursuer and target
    theta = y(2);     % LOS angle
    V_theta = y(3);   % Angular velocity (rate of change of LOS angle)
    V_R = y(4);       % Radial velocity (rate of change of range)
    alpha_P = y(5);   % Pursuer heading angle
    alpha_T = y(6); 

    global N_values t_ori; 

    %%Look Angle Constraint
    mu_max = deg2rad(45);
    mu = alpha_P - theta;

    if (alpha_P_df - alpha_P)/(alpha_P_df - theta) < 2
        N = N_ori;
        if abs(mu) >= mu_max
            N=1;
        end  
    else
        %N = (alpha_P_df - alpha_P)/(alpha_P_df - theta);
        N = 2;
%       if mean(N_values) > N_ori 
%             mean(N_values)
        if t_ori == 0
            t_ori = t;
        end
        if abs(mu) >= mu_max
            N=1;
        end  
    end  
    N_values = [N_values; N];

    vars = [V_P; V_T; alpha_P_df; N] ;   
end