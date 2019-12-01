%% PROBLEM SETUP
%{
1. dt = 0.05;
2. Npred = 20;
3. Tspan = 0:dt:20;
input 
%}


%% SETUP

close all; clear variables; clc;

% Constants

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

% dubConnObj = dubinsConnection;
% 
% startPose = [0 -105 0];
% goalPose = [0 105 pi];
% 
% pathSegObj = connect(dubConnObj,startPose,goalPose);
% 
% dubConnObj.MinTurningRadius = 105;
% 
% [pathSegObj, pathCosts] = connect(dubConnObj,startPose,goalPose);
% pathSegObj{1}.MotionTypes

% figure;
% show(pathSegObj{1})
% 
% len = pathSegObj{1}.Length;
% 
% constVel = 5;
% dt = 0.01;
% dlen = constVel*dt;
% 
% poses = interpolate(pathSegObj{1},0:dlen:len);
% figure;
% quiver(poses(:,1),poses(:,2),cos(poses(:,3)),sin(poses(:,3)),0.5)

% delta reference calculation

% delta_ref = ones(1, length(poses))*0.02635; % delta_ref to follow a curve of constant radius  
    
% test reference

% z0 = [0, -105, 0];
% z = zeros(3, length(poses));
% z(:, 1) = z0;
% 
% for i = 1:1:length(poses)-1
%     z(:, i+1) = z(:, i) + dt * model(z(:, i), constVel, delta_ref(i), b, L);
% end
% 
% 
% figure;
% plot(z(1, :), z(2, :), 'r');
% hold on;
% plot(path.xCL, path.yCL, 'g'); 
% title('reference trajectory');
% hold off;


%% lane change maneuver
dubConnObj = dubinsConnection;

% at 5 m/s this lane change takes like 2s
startPose = [0 0 0];
goalPose = [10 3 0];

pathSegObj = connect(dubConnObj,startPose,goalPose);

dubConnObj.MinTurningRadius = 1;

[pathSegObj, pathCosts] = connect(dubConnObj,startPose,goalPose);
pathSegObj{1}.MotionTypes

% figure;
% show(pathSegObj{1})

len = pathSegObj{1}.Length;

constVel = 5;
dt = 0.01;
dlen = constVel*dt;

poses = interpolate(pathSegObj{1},0:dlen:len);
% figure;
% quiver(poses(:,1),poses(:,2),cos(poses(:,3)),sin(poses(:,3)),0.5)

delta_ref = zeros(1,length(poses));

delta_ref = deg2rad(delta_ref);
% manual obtained steering command for a lane change maneuver
delta_ref(1) = 0.06;
delta_ref(2) = 0.13;
delta_ref(3) = 0.22;
delta_ref(4) = 0.32;
delta_ref(5) = 0.4;
delta_ref(6) = 0.47; % .45
delta_ref(7) = 0.47;
delta_ref(8) = 0.47;
delta_ref(9) = 0.47;
delta_ref(10) = 0.46;
delta_ref(11) = 0.44;
delta_ref(12) = 0.42;
delta_ref(13) = 0.4;
delta_ref(14) = 0.39;
delta_ref(15) = 0.38;
delta_ref(16) = 0.37;
delta_ref(17) = 0.35;
delta_ref(18) = 0.34;
delta_ref(19) = 0.33;
delta_ref(20) = 0.32;
delta_ref(21) = 0.31;
delta_ref(22) = 0.3;
delta_ref(23) = 0.29;
delta_ref(24) = 0.28;
delta_ref(25) = 0.27;
delta_ref(26) = 0.27;
delta_ref(27) = 0.26;
delta_ref(28) = 0.24;
delta_ref(29) = 0.23;
delta_ref(30) = 0.22;
delta_ref(31) = 0.22;
delta_ref(32) = 0.21;
delta_ref(33) = 0.20;
delta_ref(33) = 0.20;
delta_ref(34:36) = 0.19;
delta_ref(37) = 0.18;
delta_ref(38:40) = 0.17;
delta_ref(41) = 0.16;
delta_ref(42) = 0.15;
delta_ref(43) = 0.14;
delta_ref(44:49) = 0.13;
delta_ref(50) = 0.12;
delta_ref(51) = 0.11;
delta_ref(52) = 0.10;
delta_ref(53) = 0.09;
delta_ref(54) = 0.09;
delta_ref(55) = 0.09;
delta_ref(56) = 0.1;
delta_ref(57) = 0.09;
delta_ref(58) = 0.08;
delta_ref(59:60) = 0.08;
delta_ref(61:63) = 0.08;
delta_ref(64) = 0.07;
delta_ref(65) = 0.06;
delta_ref(66:72) = 0.06;
delta_ref(73:90) = 0.04;
delta_ref(91:100) = 0.03;
delta_ref(101:105) = 0.02;
delta_ref(106:110) = 0.01;
delta_ref(111:128) = 0;
delta_ref(129:144) = 0.01;
delta_ref(145:150) = 0.005;
delta_ref(151:167) = 0;
delta_ref(168:182) = 0.005;
delta_ref(183:202) = -0.002;
delta_ref(203) = -0.01;
delta_ref(204) = -0.1;
delta_ref(205) = -0.2;
delta_ref(206) = -0.25;
delta_ref(207) = -0.35;
delta_ref(208) = -0.39;
delta_ref(209) = -0.45;
delta_ref(210) = -0.48;
delta_ref(211) = -0.48;
delta_ref(212) = -0.48;

% Plot reference
z = zeros(3, length(poses));
%delta_ref = ones(1, length(poses))*0.02635;   % 0.02635

for i = 1:1:length(poses)-1
    z(:, i+1) = z(:, i) + dt * model(z(:, i), constVel, delta_ref(i), b, L);
end

figure;
plot(poses(:,1),poses(:,2));
hold on
plot(z(1, :), z(2, :), 'r');
legend('reference','actual');
% figure;
% plot(z(1, :), z(2, :), 'r');
% title('reference trajectory');
% hold off;

z = z';

%% test MPC without obstacles

%% Model Setup
Y_ref = poses';
U_ref = [constVel*ones(1, length(poses));...
    delta_ref];

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




%% functions for constraint generation and dynamics


function [Aeq,beq]=eq_cons(initial_idx,A,B,x_initial,npred,nstates,ninputs)
%build matrix for A_i*x_i+B_i*u_i-x_{i+1}=0
%in the form Aeq*z=beq
%initial_idx specifies the time index of initial condition from the reference trajectory 
%A and B are function handles above

%initial condition
x_initial=x_initial(:);

%size of decision variable and size of part holding states
zsize=(npred+1)*nstates+npred*ninputs;
xsize=(npred+1)*nstates;

Aeq=zeros(xsize,zsize);
Aeq(1:nstates,1:nstates)=eye(nstates); %initial condition 
beq=zeros(xsize,1);
beq(1:nstates)=x_initial;

state_idxs=nstates+1:nstates:xsize;
input_idxs=xsize+1:ninputs:zsize;

for i=1:npred
    %negative identity for i+1
    Aeq(state_idxs(i):state_idxs(i)+nstates-1,state_idxs(i):state_idxs(i)+nstates-1)=-eye(nstates);
    
    %A matrix for i
    Aeq(state_idxs(i):state_idxs(i)+nstates-1,state_idxs(i)-nstates:state_idxs(i)-1)=A(initial_idx+i-1);
    
    %B matrix for i
    Aeq(state_idxs(i):state_idxs(i)+nstates-1,input_idxs(i):input_idxs(i)+ninputs-1)=B(initial_idx+i-1);
end

end

function [Lb,Ub]=bound_cons(initial_idx,U_ref,npred,input_range,nstates,ninputs)
%time_idx is the index along uref the initial condition is at
xsize=(npred+1)*nstates;
usize=npred*ninputs;

Lb=[];
Ub=[];
for j=1:ninputs
Lb=[Lb;input_range(j,1)-U_ref(j,initial_idx:initial_idx+npred-1)];
Ub=[Ub;input_range(j,2)-U_ref(j,initial_idx:initial_idx+npred-1)];
end

Lb=reshape(Lb,[usize,1]);
Ub=reshape(Ub,[usize,1]);

Lb=[-Inf(xsize,1);Lb];
Ub=[Inf(xsize,1);Ub];

end

function dzdt=kinematic_bike_dynamics(t,z,U_in,T,b,L)

if length(T)<=1 || isempty(T) || size(U_in,2)==1
    delta=U_in(2);
    u=U_in(1);
else
    delta=interp1(T',U_in(2,:)',t,'previous');
    u=interp1(T',U_in(1,:)',t,'previous');
end

dzdt=[u*cos(z(3))-b/L*u*tan(delta)*sin(z(3));...
      u*sin(z(3))+b/L*u*tan(delta)*cos(z(3));...
      u/L*tan(delta)];

end

function zdot = model(z, u, delta, b, L)

  x = z(1);
  y = z(2);
  psi = z(3);

  zdot = zeros(3, 1);

  zdot(1) = u * cos(psi) - (b/L) * u * tan(delta) * sin(psi);
  zdot(2) = u * sin(psi) + (b/L) * u * tan(delta) * cos(psi);
  zdot(3) = (u/L) * tan(delta);

end
