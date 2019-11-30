function [g,h,dg,dh] = nonlcon(z,numSteps,dt,numStates,numInputs,innerRadius,outerRadius,x_c,y_c,b,L)
    % size of inequality cons is 401 * 1
    % states vector is 3x1, input vector is 2x1
    totalStates = numStates * numSteps;
    totalInputs = numInputs * numSteps;
    numConstraints = 2;
    
%     g = zeros(numSteps, numConstraints); % 401 * 2 
    g1 = zeros(numSteps, 1);
    g2 = zeros(numSteps, 1);
    h = zeros(totalStates, 1);
    % for 2 constraints, dg should have twice as many columns (after
    % transposing)
    dg1 = zeros(numSteps, totalStates + totalInputs - numInputs);
    dg2 = zeros(numSteps, totalStates + totalInputs - numInputs);
    dh = zeros(totalStates, totalStates + totalInputs - numInputs);
    
    x = @(i) z((i) * numStates + 1);
    y = @(i) z((i) * numStates + 2);
    psi = @(i) z((i) * numStates + 3);
    u = @(i) z(totalStates + (numInputs * i + 1));
    delta = @(i) z(totalStates + (numInputs * i + 2));
    
    xdot = @(i) (u(i)*cos(psi(i)) - b/L*u(i)*tan(delta(i))*sin(psi(i)));
    ydot = @(i) (u(i)*sin(psi(i)) + b/L*u(i)*tan(delta(i))*cos(psi(i)));
    psidot = @(i) (u(i)/L)*tan(delta(i));
    
    
    % Build inequality constraints
    % 1st constraint - outside of inner circle
    % 2nd constraint - inside of outer circle
    for i = 0:numSteps - 1
        % Define 1st constraint in 1st column
        g1(i + 1, 1) = innerRadius^2 - (x(i) - x_c)^2 - (y(i) - y_c)^2;
        % Define 2nd constraint in 1st column
        g2(i + 1, 1) = (x(i) - x_c)^2 + (y(i) - y_c)^2 - outerRadius^2;
    end
    
    g = [g1;g2];
    
    % Build dg based on gradient of the constraints
    % Only x and y are included in the constraints
    for i = 0:numSteps - 1
        dg1(i+1, ((i)*numStates+1):((i)*numStates+2)) = ...
            [(-2*(x(i)-x_c)), (-2*(y(i)-y_c))];
        dg2(i+1, ((i)*numStates+1):((i)*numStates+2)) = ...
                [(2*(x(i)-x_c)), (2*(y(i)-y_c))];
    end
    
    dg = [dg1; dg2];
    
    dg = dg';
    
    h(1:numStates,1) = [x(0) y(0) psi(0)]; 
    
    for i = 1:numSteps - 1
        h(numStates * i + 1) = x(i) - x(i-1) - dt*xdot(i-1);
        h(numStates * i + 2) = y(i) - y(i-1) - dt*ydot(i-1);
        h(numStates * i + 3) = psi(i) - psi(i-1) - dt*psidot(i-1);
    end
    
    dh(1, 1) = 1;
    dh(2, 2) = 1;
    dh(3, 3) = 1;
    
    for i = 1:numSteps - 1
        
        dh((numStates*i+1), (numStates*i+1)) = 1;               % x(i)
        dh((numStates*i+1), (numStates*(i-1)+1)) = -1;                          % x(i-1)
        dh((numStates*i+1), (numStates*(i-1)+3)) = dt*u(i-1)*sin(psi(i-1))...   % psi(i-1)
            +0.5*dt*u(i-1)*tan(delta(i-1))*cos(psi(i-1));
        
        dh((numStates*i+2), (numStates*i+2)) = 1;                               % y(i)
        dh((numStates*i+2), (numStates*(i-1)+2)) = -1;                          % y(i-1)
        dh((numStates*i+2), (numStates*(i-1)+3)) = -dt*(u(i-1)*cos(psi(i-1))... % psi(i-1)
            -0.5*u(i-1)*tan(delta(i-1))*sin(psi(i-1)));
        
        dh((numStates*i+3), (numStates*i+3)) = 1;                               % psi(i)
        dh((numStates*i+3), (numStates*(i-1)+3)) = -1;                          % psi(i-1)
        
    end
    
    for i = 1:numSteps - 1
        dh((numStates*i+1), (totalStates+2*(i-1)+1)) = -dt*(cos(psi(i-1))...          % u(i-1)
            -0.5*tan(delta(i-1))*sin(psi(i-1)));
        dh((numStates*i+1), (totalStates+2*(i-1)+2)) = dt*0.5*u(i-1)*...              % delta(i-1)
            (sec(delta(i-1)))^2*sin(psi(i-1));
        
        dh((numStates*i+2), (totalStates+2*(i-1)+1)) = -dt*(sin(psi(i-1))...          % u(i-1)
            +0.5*tan(delta(i-1))*cos(psi(i-1)));
        dh((numStates*i+2), (totalStates+2*(i-1)+2)) = -dt*0.5*u(i-1)*...             % delta(i-1)
            (sec(delta(i-1)))^2*cos(psi(i-1));
        
        dh((numStates*i+3), (totalStates+2*(i-1)+1)) = -dt/3*tan(delta(i-1));         % u(i-1)
        dh((numStates*i+3), (totalStates+2*(i-1)+2)) = -dt*u(i-1)/3*(sec(delta(i-1)))^2; %del(i-1)
    end
    
    dh = dh';
end
