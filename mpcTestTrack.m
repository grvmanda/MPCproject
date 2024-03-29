%% PROBLEM SETUP
%{
1. dt = 0.05;
2. Npred = 20;
3. Tspan = 0:dt:20;
input 
%}


%% SETUP

close all; clear variables; clc;

%% Constants

% Circular track
r1 = 100;
r2 = 150;

% Car Parameters

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


options = optimoptions('fmincon','SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true);

xFinal = 0;
yFinal = (r1+r2)/2;
psiFinal = pi;
numStates = 3;
numInputs = 2;

dt = 0.05;
Tspan = 0:dt:60;
numSteps = length(Tspan);

x_c = 0;
y_c = 0;


ub = [repmat([151 151 1.2*pi]', [numSteps 1]);...
    repmat([10 0.5]', [numSteps-1 1])];

lb = [repmat([0 -150 -1.2*pi]', [numSteps 1]);...
    repmat([0 -0.5]', [numSteps-1 1])];


cf = @(z) costfun(z, numSteps, xFinal, yFinal, psiFinal);
nc = @(z) nonlcon(z,numSteps,dt,numStates,numInputs,r1,r2,x_c,y_c,b,L);

x0 = [0, -(r1+r2)/2, 0];
z0 = zeros(1,5*(numSteps)-2);
z0(1:3) = x0; 
Z_ref = fmincon(cf, z0,[],[],[],[],lb',ub',nc,options);


Y0=reshape(Z_ref(1:3*numSteps),3,numSteps)';
U=reshape(Z_ref(3*numSteps+1:end),2,numSteps-1);

figure;
plot(Y0(:,1), Y0(:,2), '*');




%% MPC Problem setup
% 





%% FUNCTIONS

function zdot = model(z, u, delta, b, L)

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