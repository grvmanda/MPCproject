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
T = 0:dt:dt*(length(poses)-1);

numObs = 7;

Xobs = generateRandomObstacles(numObs,path);

Y = [];
U = [];

turningCurrently = 0;

while true
    [Yt,Ut, turn] = runMPC(input_range,npred,length(T),Y_ref,U_ref,A,B,Xobs,path,turningCurrently);
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
    else
        if (turn == 1)      % turn to right lane
            turningCurrently = 1;
            startPose = Yt(:,end)';
            dist_travelled = 10;
            theta_diff = dist_travelled/rightLaneCenterR;
            new_theta = Yt(3,end)+theta_diff;
            goalPose = [rightLaneCenterR*sin(new_theta),...
                -rightLaneCenterR*cos(new_theta),...
                new_theta];
            [poses, ~] = ref_traj_gen(startPose, goalPose, 5);
            
            nextStartPose = goalPose;
            nextGoalPose = [0 rightLaneCenterR pi];
            
            computeModelHandles(poses);
            
            T = 0:dt:dt*(length(poses)-1);
        elseif (turn == 2)  % turn to left lane
            turningCurrently = 1;
            startPose = Yt(:,end)';
            dist_travelled = 10;
            theta_diff = dist_travelled/leftLaneCenterR;
            new_theta = Yt(3,end)+theta_diff;
            goalPose = [leftLaneCenterR*sin(new_theta), ...
                -leftLaneCenterR*cos(new_theta), ...
                new_theta];
            [poses, ~] = ref_traj_gen(startPose, goalPose, 5);
            
            nextStartPose = goalPose;
            nextGoalPose = [0 leftLaneCenterR pi];
            
            computeModelHandles(poses);
            
            T = 0:dt:dt*(length(poses)-1);
        else
            turningCurrently = 0;
            
            [poses, ~] = ref_traj_gen(nextStartPose, nextGoalPose, nextGoalPose(2));
            computeModelHandles(poses);
            
            T = 0:dt:dt*(length(poses)-1);
        end
    end
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

hold off



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
