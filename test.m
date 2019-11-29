clear variables; close all; clc

% Test on Straight Path

% Define Vehicle Parameters
a = 1.35;
b = 1.45;
L = a+b;

% Define Simulation Parameters
dt = 0.05;
Tspan = 0:dt:10;
numSteps = length(Tspan);

xFinal = 10;
yFinal = 10;
psiFinal = 0;
numStates = 3;
numInputs = 2;

options = optimoptions('fmincon','SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true);

ub = [repmat([11 11 pi]', [numSteps 1]);...
    repmat([4 0.5]', [numSteps-1 1])];

lb = [repmat([0 0 -pi]', [numSteps 1]);...
    repmat([0 -0.5]', [numSteps-1 1])];

cf = @(z) costfun(z, numSteps, xFinal, yFinal, psiFinal);
nc = @(z) nonLin(z,numSteps,dt,numStates,numInputs,b,L);
x0 = [1E-10, 1E-10, 1E-10];
z0 = zeros(1,5*(numSteps)-2);
z0(1:3) = x0; 
Z_ref = fmincon(cf, z0,[],[],[],[],lb',ub',nc,options);


Y0=reshape(Z_ref(1:3*numSteps),3,numSteps)';
U=reshape(Z_ref(3*numSteps+1:end),2,numSteps-1);

figure;
plot(Y0(:,1), Y0(:,2), '*');



function [g,h,dg,dh] = nonLin(z,numSteps,dt,numStates,numInputs,b,L)
    % size of inequality cons is 401 * 1
    % states vector is 3x1, input vector is 2x1
    totalStates = numStates * numSteps;
    totalInputs = numInputs * numSteps;
    
    g = [];
    dg = [];
    
    %numConstraints = 2;
    %g = zeros(numSteps, numConstraints); % 401 * 2 (just one constraint for now)
    h = zeros(totalStates, 1);
    dh = zeros(totalStates, totalStates + totalInputs - numInputs);
    
    x = @(i) z((i) * numStates + 1);
    y = @(i) z((i) * numStates + 2);
    psi = @(i) z((i) * numStates + 3);
    u = @(i) z(totalStates + (numInputs * i + 1));
    delta = @(i) z(totalStates + (numInputs * i + 2));
    
    xdot = @(i) (u(i)*cos(psi(i)) - b/L*u(i)*tan(delta(i))*sin(psi(i)));
    ydot = @(i) (u(i)*sin(psi(i)) + b/L*u(i)*tan(delta(i))*cos(psi(i)));
    psidot = @(i) (u(i)/L)*tan(delta(i));
    
%     % Build inequality constraints
%     % 1st constraint - outside of inner circle
%     % 2nd constraint - inside of outer circle
%     for i = 0:numSteps - 1
%         % Define 1st constraint in 1st column
%         g(i + 1, 1) = innerRadius^2 - (x(i) - x_c)^2 - (y(i) - y_c)^2;
%         % Define 2nd constraint in 1st column
%         g(i + 1, 2) = (x(i) - x_c)^2 + (y(i) - y_c)^2 - outerRadius^2;
%     end
    
    h(1:numStates,1) = [x(0) y(0) psi(0)]; % no equality constraints
    
    for i = 1:numSteps - 1
        h(numStates * i + 1) = x(i) - x(i-1) - dt*xdot(i-1);
        h(numStates * i + 2) = y(i) - y(i-1) - dt*ydot(i-1);
        h(numStates * i + 3) = psi(i) - psi(i-1) - dt*psidot(i-1);
    end
    
    dh(1, 1) = 1;
    dh(2, 2) = 1;
    dh(3, 3) = 1;
    
    for i = 1:numSteps - 1
        
        dh((3*i+1), (3*i+1)) = 1;                               % x(i)
        dh((3*i+1), (3*(i-1)+1)) = -1;                          % x(i-1)
        dh((3*i+1), (3*(i-1)+3)) = 0.05*u(i-1)*sin(psi(i-1))... % psi(i-1)
            +0.5*0.05*u(i-1)*tan(delta(i-1))*cos(psi(i-1));
        
        dh((3*i+2), (3*i+2)) = 1;                               % y(i)
        dh((3*i+2), (3*(i-1)+2)) = -1;                          % y(i-1)
        dh((3*i+2), (3*(i-1)+3)) = -0.05*(u(i-1)*cos(psi(i-1))... % psi(i-1)
            -0.5*u(i-1)*tan(delta(i-1))*sin(psi(i-1)));
        
        dh((3*i+3), (3*i+3)) = 1;                               % psi(i)
        dh((3*i+3), (3*(i-1)+3)) = -1;                          % psi(i-1)
        
    end
    
    for i = 1:numSteps - 1
        dh((3*i+1), (totalStates+2*(i-1)+1)) = -0.05*(cos(psi(i-1))...          % u(i-1)
            -0.5*tan(delta(i-1))*sin(psi(i-1)));
        dh((3*i+1), (totalStates+2*(i-1)+2)) = 0.05*0.5*u(i-1)*...              % delta(i-1)
            (sec(delta(i-1)))^2*sin(psi(i-1));
        
        dh((3*i+2), (totalStates+2*(i-1)+1)) = -0.05*(sin(psi(i-1))...          % u(i-1)
            +0.5*tan(delta(i-1))*cos(psi(i-1)));
        dh((3*i+2), (totalStates+2*(i-1)+2)) = -0.05*0.5*u(i-1)*...             % delta(i-1)
            (sec(delta(i-1)))^2*cos(psi(i-1));
        
        dh((3*i+3), (totalStates+2*(i-1)+1)) = -0.05/3*tan(delta(i-1));         % u(i-1)
        dh((3*i+3), (totalStates+2*(i-1)+2)) = -0.05*u(i-1)/3*(sec(delta(i-1)))^2; %del(i-1)
    end
    
    dh = dh';
end
