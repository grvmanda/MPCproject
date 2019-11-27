function [g, h] = nonlcon(z,N,dt,numStates,innerRadius,outerRadius,x_c,y_c,b,L)
    % size of inequality cons is 401 * 1
    % states vector is 3x1, input vector is 2x1
    numSteps = N / dt + 1;
    numConstraints = 2;
    g = zeros(numSteps, numConstraints); % 401 * 2 (just one constraint for now)
    h = zeros(numStates * numSteps, 1);
    
    x = @(i) z((i) * numStates + 1);
    y = @(i) z((i) * numStates + 2);
    psi = @(i) z((i) * 3 + 3);
    delta = @(i) z(numSteps * numStates + (numInputs * i + 1));
    u = @(i) z(numSteps * numStates + (numInputs * i + 2));
    
    xdot = @(i) (u(i)*cos(psi(i)) - b/L*u(i)*tan(delta(i))*sin(psi(i)));
    ydot = @(i) (u(i)*sin(psi(i)) + b/L*u(i)*tan(delta(i))*cos(psi(i)));
    psidot = @(i) (u(i)/L)*tan(delta(i));
    
    % Build inequality constraints
    % 1st constraint - outside of inner circle
    % 2nd constraint - inside of outer circle
    for i = 0:numSteps - 1
        % Define 1st constraint in 1st column
        g(i + 1, 1) = innerRadius^2 - (x(i) - x_c)^2 - (y(i) - y_x)^2;
        % Define 2nd constraint in 1st column
        g(i + 1, 2) = (x(i) - x_c)^2 + (y(i) - y_c)^2 - outerRadius^2;
    end
    
    h(1:numStates,1) = [x(0) y(0) psi(0)]; % no equality constraints
    
    for i = 1:numSteps
        h(numStates * i + 1) = x(i) - x(i-1) - dt*xdot(i-1);
        h(numStates * i + 2) = y(i) - y(i-1) - dt*ydot(i-1);
        h(numStates * i + 3) = psi(i) - psi(i-1) - dt*psidot(i-1);
    end
end
