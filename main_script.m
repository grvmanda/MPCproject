%% SETUP

close all; clear all; clc;

%% Constants

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
Y_ref = poses';
U_ref = [constVel*ones(1, length(poses));...
    delta_ref_complete];

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

%final trajectory
Y=NaN(3,length(T));

%applied inputs
U=NaN(2,length(T));

%input from QP
u_mpc=NaN(2,length(T));

%error in states (actual-reference)
eY=NaN(3,length(T));

%set random initial condition
eY0=[0;-0.25;0];
Y(:,1)=Y_ref(:,1)-eY0;

for i=1:length(T)-1
    %shorten prediction horizon if we are at the end of trajectory
    npred_i=min([npred,length(T)-i]);
    
    %calculate error
    eY(:,i)=Y(:,i)-Y_ref(:,i);

    %generate equality constraints
    [Aeq,beq]=eq_cons(i,A,B,eY(:,i),npred_i,nstates,ninputs);
    
    %generate boundary constraints
    [Lb,Ub]=bound_cons(i,U_ref,npred_i,input_range,nstates,ninputs);
    
    %cost for states
    Q=[1,1,0.5];
    
    %cost for inputs
    R=[0.1,0.01];
    
    H=diag([repmat(Q,[1,npred_i+1]),repmat(R,[1,npred_i])]);
    
    f=zeros(nstates*(npred_i+1)+ninputs*npred_i,1);
    
    [x,fval] = quadprog(H,f,[],[],Aeq,beq,Lb,Ub);
    
    %get linearized input
    u_mpc(:,i)=x(nstates*(npred_i+1)+1:nstates*(npred_i+1)+ninputs);
    
    %get input
    U(:,i)=u_mpc(:,i)+U_ref(:,i);
    
    
    %simulate model
    [~,ztemp]=ode45(@(t,z)kinematic_bike_dynamics(t,z,U(:,i),0,b,L),[0 dt],Y(:,i));
    
    %store final state
    Y(:,i+1)=ztemp(end,:)';
end

figure;
plot(Y_ref(1,:),Y_ref(2,:))
hold on
xlabel('x [m]')
ylabel('y [m]')
plot(Y(1,:),Y(2,:))
plot(path.xL, path.yL, 'k', path.xR, path.yR, 'k', ...
    path.xCL, path.yCL, 'b');
hold off

figure;
subplot(3,1,1)
plot(Y_ref(1,:),Y_ref(2,:))
hold on
xlabel('x [m]')
ylabel('y [m]')
subplot(3,1,2)
plot(Y_ref(1,:),U_ref(1,:))
hold on
xlabel('x [m]')
ylabel('u [m/s]')
subplot(3,1,3)
plot(Y_ref(1,:),U_ref(2,:))
hold on
xlabel('x [m]')
ylabel('\delta_f [rad]')

subplot(3,1,1)
plot(Y(1,:),Y(2,:))
subplot(3,1,2)
plot(Y_ref(1,:),U(1,:))
subplot(3,1,3)
plot(Y_ref(1,:),U(2,:))

