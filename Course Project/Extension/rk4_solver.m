
function y = rk4_solver(equations, conditions, y0, x, dx, params, terminal_condition)
    % RK4 solver for N-dimensional state vector with terminal condition
    % equations: function handle for equations of the form dydt = f(x, y, params)
    % y0: initial state vector [y1_0, y2_0, ..., yN_0]
    % x: array of x values (e.g., time or space points)
    % dx: step size
    % params: additional parameters for the equations
    % terminal_condition: function handle for checking terminal condition (e.g., @(y) y(1) > threshold)

    N = length(y0); % Number of states
    M = length(x); % Number of steps

    % Initialize state vector
    y = zeros(M, N);
    y(1, :) = y0; % Set initial conditions

    % Fourth-order Runge-Kutta method
    for i = 1:M-1
        k1 = equations(x(i), y(i, :)', params);
        k2 = equations(x(i) + 0.5*dx, y(i, :)' + 0.5*dx*k1, params);
        k3 = equations(x(i) + 0.5*dx, y(i, :)' + 0.5*dx*k2, params);
        k4 = equations(x(i) + dx, y(i, :)' + dx*k3, params);
        
        % Update state vector
        y(i+1, :) = y(i, :) + (dx/6) * (k1' + 2*k2' + 2*k3' + k4');
        
        % Check terminal condition
        if terminal_condition(y(i+1, :))
            y = y(1:i+1, :); % Truncate output to the current step
            break;
        end

        params = conditions(x(i), y(i,:), params);
    end
end