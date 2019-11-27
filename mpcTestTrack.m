%% PROBLEM SETUP
%{

1. dt = 0.05;
2. Npred = 20;
3. Tspan = 0:dt:20;

input 

%}


%% SETUP

close all; clear all;

%% Constants

% Circular track
r1 = 100;
r2 = 150;

% Car Parameters

global a b L

a = 1.35;
b = 1.45;
L = a+b;


%% PATH GEN
% Path is a semi-circle for now.
% r1 is radius of inner circle
% r2 is radius of outer circle
path.numPoints = 1000;
thetaVal = linspace(-pi/2, pi/2, path.numPoints);
  
path.xL = r1*cos(thetaVal);
path.xR = r2*cos(thetaVal);
path.yL = r1*sin(thetaVal);
path.yR = r2*sin(thetaVal);
path.xCL = ((r1 + r2)/2)*cos(thetaVal);
path.yCL = ((r1 + r2)/2)*sin(thetaVal);

figure;
plot(path.xL, path.yL, 'k', path.xR, path.yR, 'k', ...
    path.xCL, path.yCL, ' :r');


%% Trajectory Generation
% Constraints: 











%% MPC Problem setup
% 





%% FUNCTIONS

function zdot = model(z, u, delta)

  global b L

  x = z(1);
  y = z(2);
  psi = z(3);

  zdot = zeros(3, 1);

  zdot(1) = u * cos(psi) - (b/L) * u * tan(delta) * sin(psi);
  zdot(2) = u * sin(psi) + (b/L) * u * tan(delta) * cos(psi);
  zdot(3) = (u/L) * tan(delta);

end


%function [g, dg, h, dg] = nlcon(z)

  


%end



function [J, dJ] = costfun(z, nsteps, xFinal, yFinal, psiFinal)

    % size of J must be 1 x 1
    x = @(i) z((i)*3+1);
    y = @(i) z((i)*3+2);
    psi = @(i) z((i)*3+3);
    u = @(i) z((nsteps+1)*3+(2*i+1));
    delta = @(i) z((nsteps+1)*3+(2*i+2));

    J = 0;
    for i = 0:nsteps
        J = J + (x(i)-xFinal)^2 + (y(i)-yFinal)^2 + (psi(i)-psiFinal)^2;
    end

    for j = 0:nsteps-1
        J = J + u(j)^2 + delta(j)^2;
    end
    
    dJ = zeros(1, 3*(nsteps+1)+2*nsteps);

    for i = 0:nsteps
        dJ(3*(i)+1) = 2*(x(i)-xFinal);
        dJ(3*(i)+2) = 2*(y(i)-yFinal;
        dJ(3*(i)+3) = 2*(psi(i)-psiFinal);
    end

    for j = 0:nsteps-1
        dJ(3*(nsteps+1)+2*j+1) = 2*u(j);
        dJ(3*(nsteps+1)+2*j+2) = 2*delta(j);
    end

end
