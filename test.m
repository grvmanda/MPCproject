clear variables; close all; clc

% Define Vehicle Parameters
a = 1.35;
b = 1.45;
L = a+b;

% Circular track
r1 = 100;
r2 = 110;

x_c = 0;
y_c = 105;

% PATH GEN
% Path is a semi-circle for now.
% r1 is radius of inner circle
% r2 is radius of outer circle
path.numPoints = 1000;
thetaVal = linspace(-pi/2, pi/2, path.numPoints);
  
path.xL = r1*cos(thetaVal) + x_c;
path.xR = r2*cos(thetaVal) + x_c;
path.yL = r1*sin(thetaVal) + y_c;
path.yR = r2*sin(thetaVal) + y_c;
path.xCL = ((r1 + r2)/2)*cos(thetaVal) + x_c;
path.yCL = ((r1 + r2)/2)*sin(thetaVal) + y_c;

figure;
plot(path.xL, path.yL, 'k', path.xR, path.yR, 'k', ...
    path.xCL, path.yCL, ' :r');

% Define Simulation Parameters
dt = 0.05;
Tspan = 0:dt:20;
numSteps = length(Tspan);

xFinal = 40;
yFinal = 6;
% xFinal = 7;
% yFinal = 0;
psiFinal = 0;
numStates = 3;
numInputs = 2;

options = optimoptions('fmincon','SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true,'PlotFcn','optimplotfunccount');

ub = [repmat([41 13.5 pi/2]', [numSteps 1]);...
    repmat([1 0.5]', [numSteps-1 1])];

lb = [repmat([-1 -3 -pi/2]', [numSteps 1]);... % lower bound on x y is crucial
    repmat([0 -0.5]', [numSteps-1 1])];

% ub = [repmat([8 3 pi/2]', [numSteps 1]);...
%     repmat([1 0.5]', [numSteps-1 1])];
% 
% lb = [repmat([-1 -3 -pi/2]', [numSteps 1]);... % lower bound on x y is crucial
%     repmat([0 -0.5]', [numSteps-1 1])];

cf = @(z) costfun(z, numSteps, xFinal, yFinal, psiFinal);
nc = @(z) nonLin(z,numSteps,dt,numStates,numInputs,r1,r2,x_c,y_c,b,L);
x0 = [0, 0, 0]';
%x0 = [0, 0, 0]';
z0 = zeros(1,5*(numSteps)-2);
z0(1:3) = x0; 
Z_ref = fmincon(cf, z0,[],[],[],[],lb',ub',nc,options);


Y0=reshape(Z_ref(1:3*numSteps),3,numSteps)';
U=reshape(Z_ref(3*numSteps+1:end),2,numSteps-1);

figure;
%plot(Y0(:,1), Y0(:,2), '*');

% write input function as zero order hold
u = @(t) [interp1(0:dt:(numSteps-2)*dt,U(1,:),t,'previous','extrap');...
       interp1(0:dt:(numSteps-2)*dt,U(2,:),t,'previous','extrap')] ;
   
% run ode45 for each initial condition
[T1,Y1] = ode45(@(t,x) odefun(x,u(t)),[0:dt:(numSteps-1)*dt],[0 0 0]) ;

% plot trajectories obstacle and initial condition
plot(Y0(:,1),Y0(:,2),Y1(:,1),Y1(:,2),0,0,'x') ;

legend('fmincon trajectory','ode45 trajectory using x0 = [0;0;0]','start') ;
%ylim([-2,2]); xlim([-1,8]); xlabel('x'); ylabel('y') ;


function [g,h,dg,dh] = nonLin(z,numSteps,dt,numStates,numInputs,innerRadius,outerRadius,x_c,y_c,b,L)
    % size of inequality cons is 401 * 1
    % states vector is 3x1, input vector is 2x1
    totalStates = numStates * numSteps;
    totalInputs = numInputs * numSteps;
    
    numConstraints = 2;
    g = zeros(numSteps, numConstraints); % 401 * 2 (just one constraint for now)
    h = zeros(totalStates, 1);
    dg = zeros(numConstraints*numSteps, totalStates + totalInputs - numInputs);
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
        g(i + 1, 1) = innerRadius^2 - (x(i) - x_c)^2 - (y(i) - y_c)^2;
        % Define 2nd constraint in 1st column
        g(i + 1, 2) = (x(i) - x_c)^2 + (y(i) - y_c)^2 - outerRadius^2;
    end
    
    % Build dg based on gradient of the constraints
    % Only x and y are included in the constraints
    for i = 0:numSteps - 1
        dg(i+1, ((i)*numStates+1):((i)*numStates+2)) = ...
            [(-2*(x(i)-x_c)), (-2*(y(i)-y_c))];
        dg(numSteps+i+1, ((i)*numStates+1):((i)*numStates+2)) = ...
                [(2*(x(i)-x_c)), (2*(y(i)-y_c))];
    end
    
    dg = dg';
    
%     g = [];
%     dg = [];
    
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
