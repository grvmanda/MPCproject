%% SETUP

close all; clear all; clc;

%% Constants

global a b L constVel dt

constVel = 5;
dt = 0.01;

% Car Parameters

a = 1.35;
b = 1.45;
L = a+b;

r1 = 97;
r2 = 103;

leftLaneCenterR = 98.5;
rightLaneCenterR = 101.5;

path.numPoints = 1001;
thetaVal = linspace(-pi/2, pi/2, path.numPoints);
  
path.xL = r1*cos(thetaVal);
path.xR = r2*cos(thetaVal);
path.yL = r1*sin(thetaVal);
path.yR = r2*sin(thetaVal);
path.xCL = ((r1 + r2)/2)*cos(thetaVal);
path.yCL = ((r1 + r2)/2)*sin(thetaVal);
path.theta = thetaVal;
path.cline = [path.xCL;...
    path.yCL];


%% initial trajectory

startPose = [0 -rightLaneCenterR 0];
goalPose = [0 rightLaneCenterR pi];

[poses, ~] = ref_traj_gen(startPose, goalPose, rightLaneCenterR);

%% Model Setup

global Y_ref U_ref A B x y psi u delta A_disc

computeModelHandles(poses);

%% MPC Setup

nstates=3;
ninputs=2;

%input ranges (first row is longitudinal velocity, second row is wheel angle)
input_range=[0,   5;...
            -0.5,0.5];

%11 timesteps for 3 states, 10 timesteps for 2 inputs
npred=10;
Ndec=(npred+1)*nstates+ninputs*npred;

numObs = 8;

Xobs = generateRandomObstacles(numObs,path);

% load('savedObs.mat');
% load('errorObs.mat');
load('ObsForReport.mat');

figure;
hold on
xlabel('x [m]')
ylabel('y [m]')
plot(path.xL, path.yL, 'k', path.xR, path.yR, 'k', ...
    path.xCL, path.yCL, 'b');

for i=1:numObs
    ob = Xobs{i};
    plot(ob(:,1), ob(:,2));
end

hold off




Y = [0; -rightLaneCenterR; 0];
U = [5; 0];

turningCurrently = 0;
minTurnRadius = 5;

while true
    [Yt,Ut, turn] = runMPC(input_range,npred,length(poses),Y_ref,U_ref,A,B,Xobs,path,turningCurrently,Y,U);
    Y = [Y, Yt];
    U = [U, Ut];
    
    figure;
    % plot(Y_ref(1,:),Y_ref(2,:))
    hold on
    xlabel('x [m]')
    ylabel('y [m]')
    plot(Y(1,:),Y(2,:))
    plot(path.xL, path.yL, 'k', path.xR, path.yR, 'k', ...
        path.xCL, path.yCL, 'b');

    for i=1:numObs
        ob = Xobs{i};
        plot(ob(:,1), ob(:,2));
    end

    hold off
    
    if ((Y(1,end)^2+(Y(3,end)-pi)^2)^0.5<0.2)
        break;
    elseif (turn == 0)
        turningCurrently = 0;
        [poses, ~] = ref_traj_gen(nextStartPose, nextGoalPose, nextGoalPose(2));
    else
        turningCurrently = 1;
        [poses,nextStartPose,nextGoalPose] = getTurnPoses(turn,Yt);
    end
    computeModelHandles(poses);
end

figure;
hold on
xlabel('x [m]')
ylabel('y [m]')
plot(Y(1,:),Y(2,:))
plot(path.xL, path.yL, 'k', path.xR, path.yR, 'k', ...
    path.xCL, path.yCL, 'b');

for i=1:numObs
    ob = Xobs{i};
    plot(ob(:,1), ob(:,2));
end

title('Car on track wihtout obstacles');
hold off

T = 0:0.01:0.01*(length(U)-1);

figure;
subplot(2,1,1)
plot(T, U(1, :));
title('velocity');
xlabel('time (s)');
ylabel('(m/s)');
subplot(2,1,2)
plot(T, U(2, :));
title('delta');
xlabel('time (s)');
ylabel('(rad)');



%% FUNCTIONS

function computeModelHandles(poses)

    global Y_ref U_ref A B x y psi u delta A_disc constVel dt b L
    
    Y_ref = poses';
    U_ref = [constVel*ones(1, length(poses));...
        zeros(1, length(poses))];

    x = @(i) Y_ref(1, i);
    y = @(i) Y_ref(2, i);
    psi = @(i) Y_ref(3, i);

    u = @(i) U_ref(1, i);
    delta = @(i) U_ref(2, i);

    A_disc = @(i) [0, 0, (-u(i)*sin(psi(i))-b/L*u(i)*tan(delta(i))*cos(psi(i)));...
        0, 0, (u(i)*cos(psi(i))-b/L*u(i)*tan(delta(i))*sin(psi(i)));...
        0, 0, 0];

    A = @(i) (eye(3) + dt*A_disc(i));

    B = @(i) dt*[(cos(psi(i))-b/L*tan(delta(i))*sin(psi(i))), ...
        ((-b/L)*u(i)*(sec(delta(i)))^2*sin(psi(i)));...
        (sin(psi(i))+b/L*tan(delta(i))*cos(psi(i))),...
        (b/L*u(i)*(sec(delta(i)))^2*cos(psi(i)));...
        (1/L*tan(delta(i))), (u(i)/L*(sec(delta(i)))^2)];


end
