% Given values
V_P = 1; % Pursuer speed
V_T = 1; % Target speed
nu = V_P / V_T;

Vp = linspace(-1,4,4);
x_P0 = 0; y_P0 = 1;
x_T0 = 1; y_T0 = 0;

% Alpha_P values
alpha_P = linspace(-pi/2, 0, 100);

% Initialize miss distance array
R_miss = zeros(length(Vp), length(alpha_P));

%Calculate miss distance for each alpha_P 
for j = 1:length(Vp)
    V_P = Vp(j);
    for i = 1:length(alpha_P)
        % Pursuer and target velocities
        V_Px = V_P * cos(alpha_P(i));
        V_Py = V_P * sin(alpha_P(i));
        
        a = V_T^2 - 2*V_T*V_Px + V_P^2;
        b = 2*x_T0*V_T - 2*x_T0*V_Px + 2*y_P0*V_Py;
        c = x_T0^2 + y_P0^2;
        
        % Time to minimum range
        t_min = -b/(2*a);
        
        % Ensure t_min is non-negative (future engagement/starting point is miss distance) 
        if t_min < 0
            t_min = 0;
        end
        
        % Minimum range / Miss distance
        R_miss(j, i) = sqrt(c - (b^2)/(4*a));
    end
end


% Plotting
for j = 1:length(Vp)
    %subplot(2,2,j);
    figure(j);
    plot(alpha_P, R_miss(j,:));
    xlabel('\alpha_P (radians)');
    ylabel('Miss Distance (R_{miss})');
    title(['Variation of Miss Distance with \alpha_P, \nu = ', num2str(Vp(j))]);
    grid on;
end
