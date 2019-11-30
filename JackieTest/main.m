clear variables; close all; clc
% define constants
b = 1.5 ; 
L = 3 ;
nsteps = 121 ;
dt = 0.05 ;
              
% set upper bounds
ub = zeros(5*nsteps-2,1);
ub(1:3:3*nsteps) = 8 ;
ub(2:3:3*nsteps) = 3 ;
ub(3:3:3*nsteps) = pi/2 ;
ub(3*nsteps+1:2:5*nsteps-2) = 1 ;
ub(3*nsteps+2:2:5*nsteps-2) = 0.5 ;

% set lower bounds
lb = zeros(5*nsteps-2,1);
lb(1:3:3*nsteps) = -1 ;
lb(2:3:3*nsteps) = -3 ;
lb(3:3:3*nsteps) = -pi/2 ;
lb(3*nsteps+1:2:5*nsteps-2) = 0 ;
lb(3*nsteps+2:2:5*nsteps-2) = -0.5 ;

%optimization variables
x0 = zeros(1,5*nsteps-2) ;
cf = @costfun ;
nc = @nonlcon ;
options = optimoptions('fmincon','SpecifyConstraintGradient',true,...
                       'SpecifyObjectiveGradient',true) ;
% run fmincon to find trajectory
z = fmincon(cf,x0,[],[],[],[],lb',ub',nc,options) ;

% reshape trajectory and inputs
Y0 = reshape(z(1:3*nsteps),3,nsteps)' ;
U = reshape(z(3*nsteps+1:end),2,nsteps-1) ;

% Plot Trajectories
plot(Y0(:,1),Y0(:,2),'*');

% % write input function as zero order hold
% u = @(t) [interp1(0:dt:119*dt,U(1,:),t,'previous','extrap');...
%        interp1(0:dt:119*dt,U(2,:),t,'previous','extrap')] ;
%    
% % run ode45 for each initial condition
% [T1,Y1] = ode45(@(t,x) odefun(x,u(t)),[0:dt:120*dt],[0 0 0]) ;
% [T2,Y2] = ode45(@(t,x) odefun(x,u(t)),[0:dt:120*dt],[0 0 -0.01]) ;
% 
% % plot trajectories obstacle and initial condition
% plot(Y0(:,1),Y0(:,2),Y1(:,1),Y1(:,2),Y2(:,1),Y2(:,2),...
%     (0.7*cos(0:0.01:2*pi)+3.5),(0.7*sin(0:0.01:2*pi)-0.5),0,0,'x') ;
% 
% legend('fmincon trajectory','ode45 trajectory using x0 = [0;0;0]',...
%     'ode45 trajectory using x0 = [0;0;-0.01]','buffered Obstacle','start') ;
% ylim([-2,2]); xlim([-1,8]); xlabel('x'); ylabel('y') ;








