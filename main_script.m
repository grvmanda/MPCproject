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


%% trajectory

startPose = [0 -rightLaneCenterR 0];
goalPose = [rightLaneCenterR 0 pi/2];

[poses1, delta_ref1] = ref_traj_gen(startPose, goalPose, rightLaneCenterR);

dist_travelled = 10;
theta_diff = dist_travelled/rightLaneCenterR;

startPose = [rightLaneCenterR 0 pi/2];
% goalPose = [(105*cos(theta_diff)) (0+105*sin(theta_diff)) (pi/2+theta_diff)];
goalPose = [(leftLaneCenterR*cos(theta_diff)) (0+leftLaneCenterR*sin(theta_diff)) (pi/2+theta_diff)];

[poses2, delta_ref2] = ref_traj_gen(startPose, goalPose, 5);

startPose = [(leftLaneCenterR*cos(theta_diff)) (0+leftLaneCenterR*sin(theta_diff)) (pi/2+theta_diff)];
goalPose = [0 leftLaneCenterR pi];

[poses3, delta_ref3] = ref_traj_gen(startPose, goalPose, leftLaneCenterR);

poses = [poses1;...
    poses2;...
    poses3];
delta_ref_complete = [delta_ref1*0, delta_ref2*0, delta_ref3*0];

%% Model Setup
% Y_ref = poses';
% U_ref = [constVel*ones(1, length(poses));...
%     delta_ref_complete];
% 
% x = @(i) Y_ref(1, i);
% y = @(i) Y_ref(2, i);
% psi = @(i) Y_ref(3, i);
% 
% u = @(i) U_ref(1, i);
% delta = @(i) U_ref(2, i);
% 
% A_disc = @(i) [0, 0, (-u(i)*sin(psi(i))-b/L*u(i)*tan(delta(i))*cos(psi(i)));...
%     0, 0, (u(i)*cos(psi(i))-b/L*u(i)*tan(delta(i))*sin(psi(i)));...
%     0, 0, 0];
%   
% A = @(i) (eye(3) + dt*A_disc(i));
% 
% B = @(i) dt*[(cos(psi(i))-b/L*tan(delta(i))*sin(psi(i))), ...
%     ((-b/L)*u(i)*(sec(delta(i)))^2*sin(psi(i)));...
%     (sin(psi(i))+b/L*tan(delta(i))*cos(psi(i))),...
%     (b/L*u(i)*(sec(delta(i)))^2*cos(psi(i)));...
%     (1/L*tan(delta(i))), (u(i)/L*(sec(delta(i)))^2)];

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
    if (length(Y) > 3000)
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
            [poses1, ~] = ref_traj_gen(startPose, goalPose, 5);
            
            startPose = goalPose;
            goalPose = [0 rightLaneCenterR pi];
            [poses2, ~] = ref_traj_gen(startPose, goalPose, rightLaneCenterR);
            
            poses = [poses1;poses2];
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
            [poses1, ~] = ref_traj_gen(startPose, goalPose, 5);
            
            startPose = goalPose;
            goalPose = [0 leftLaneCenterR pi];
            [poses2, ~] = ref_traj_gen(startPose, goalPose, leftLaneCenterR);
            
            poses = [poses1;poses2];
            computeModelHandles(poses);
            
            T = 0:dt:dt*(length(poses)-1);
        end
    end
end

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

% figure;
% subplot(3,1,1)
% plot(Y_ref(1,:),Y_ref(2,:))
% hold on
% xlabel('x [m]')
% ylabel('y [m]')
% subplot(3,1,2)
% plot(Y_ref(1,:),U_ref(1,:))
% hold on
% xlabel('x [m]')
% ylabel('u [m/s]')
% subplot(3,1,3)
% plot(Y_ref(1,:),U_ref(2,:))
% hold on
% xlabel('x [m]')
% ylabel('\delta_f [rad]')
% 
% subplot(3,1,1)
% plot(Y(1,:),Y(2,:))
% subplot(3,1,2)
% plot(Y_ref(1,:),U(1,:))
% subplot(3,1,3)
% plot(Y_ref(1,:),U(2,:))


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
