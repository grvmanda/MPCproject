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
r2 = 110;

% Car Parameters

a = 1.35;
b = 1.45;
L = a+b;

path.numPoints = 1001;
thetaVal = linspace(-pi/2, pi/2, path.numPoints);
  
path.xL = r1*cos(thetaVal);
path.xR = r2*cos(thetaVal);
path.yL = r1*sin(thetaVal);
path.yR = r2*sin(thetaVal);
path.xCL = ((r1 + r2)/2)*cos(thetaVal);
path.yCL = ((r1 + r2)/2)*sin(thetaVal);

%% PATH GEN
% Path is a semi-circle for now.
% r1 is radius of inner circle
% r2 is radius of outer circle

dubConnObj = dubinsConnection;

startPose = [0 -105 0];
goalPose = [0 105 pi];

pathSegObj = connect(dubConnObj,startPose,goalPose);

dubConnObj.MinTurningRadius = 105;

[pathSegObj, pathCosts] = connect(dubConnObj,startPose,goalPose);
pathSegObj{1}.MotionTypes

figure;
show(pathSegObj{1})

len = pathSegObj{1}.Length;

constVel = 5;
dt = 0.01;
dlen = constVel*dt;

poses = interpolate(pathSegObj{1},0:dlen:len);
figure;
quiver(poses(:,1),poses(:,2),cos(poses(:,3)),sin(poses(:,3)),0.5)

%% Stanley delta calculations

delta_refST = zeros(1, length(poses));

for i = 1:1:length(poses)-1
    delta_refST(i) = deg2rad(lateralControllerStanley(poses(i+1, :), ...
        poses(i, :), constVel, 'Wheelbase', L, 'PositionGain', 2.5));
end

% delta_const = deg2rad(lateralControllerStanley(poses(end, :), ...
%         poses(1, :), constVel, 'Wheelbase', L, 'PositionGain', 2.5));

delta_ref = ones(1, length(poses))*0.02635;    
    
%% test reference

z0 = [0, -105, 0];
z = zeros(3, length(poses));
z(:, 1) = z0;

for i = 1:1:length(poses)-1
    z(:, i+1) = z(:, i) + dt * model(z(:, i), constVel, delta_ref(i), b, L);
end


figure;
plot(z(1, :), z(2, :), 'r');
hold on;
plot(path.xCL, path.yCL, 'g'); 
title('reference trajectory');
hold off;


%% lane change maneuver

dubConnObj = dubinsConnection;

startPose = [0 0 0];
goalPose = [3 3 0];

pathSegObj = connect(dubConnObj,startPose,goalPose);

dubConnObj.MinTurningRadius = 1;

[pathSegObj, pathCosts] = connect(dubConnObj,startPose,goalPose);
pathSegObj{1}.MotionTypes

figure;
show(pathSegObj{1})

len = pathSegObj{1}.Length;

constVel = 5;
dt = 0.01;
dlen = constVel*dt;

poses = interpolate(pathSegObj{1},0:dlen:len);
figure;
quiver(poses(:,1),poses(:,2),cos(poses(:,3)),sin(poses(:,3)),0.5)

