
%% parameters
%wheelbases
L=3;
%distance from rear wheel to center of mass
b=1.5;

%number of states and inputs in dynamic model
nstates=3;
ninputs=2;

%input ranges (first row is longitudinal velocity, second row is wheel angle)
input_range=[0,   1;...
            -0.5,0.5];

%time discretization
dt=0.01;
%time span
T=0:dt:6;

%% load reference trajectory
load('part1_traj_05_timestep.mat')

U_ref=interp1(0:0.05:6,[U,U(:,end)]',T)';

Y_ref=interp1(0:0.05:6,Y,T)';



%% 4.1 Discrete-time A and B matrices
%these are the system linearized in discrete time about the reference
%trajectory i.e. x(i+1)=A_i*x_i+B_i*u_i

A=@(i) eye(3)+dt*[0 0 -U_ref(1,i)*sin(Y_ref(3,i))-(U_ref(1,i)*b/L)*cos(Y_ref(3,i))*tan(U_ref(2,i));...
                     0 0  U_ref(1,i)*cos(Y_ref(3,i))-(U_ref(1,i)*b/L)*sin(Y_ref(3,i))*tan(U_ref(2,i));...
                      0 0                       0                    ];
B=@(i) dt*[cos(Y_ref(3,i))-b/L*sin(Y_ref(3,i))*tan(U_ref(2,i)), -(U_ref(1,i)*b*sin(Y_ref(3,i)))/(L*cos(U_ref(2,i))^2);...
           sin(Y_ref(3,i))+b/L*cos(Y_ref(3,i))*tan(U_ref(2,i)), (U_ref(1,i)*b*cos(Y_ref(3,i)))/(L*cos(U_ref(2,i))^2);...
           1/L*tan(U_ref(2,i)),                                  U_ref(1,i)/(L*cos(U_ref(2,i))^2)];


%% 4.2 Number of decision variables for colocation method

%11 timesteps for 3 states, 10 timesteps for 2 inputs
npred=10;
Ndec=(npred+1)*nstates+ninputs*npred;
%decision variable will be z=[x_1...x_11;u_1...u_10] (x refers to state
%vector, u refers to input vector)

%% 4.3 write and test function (at index 1) to construct Aeq Beq (equality constraints
%enforce x(i+1)=A_i*x_i+B_i*u_i
eY0=[0.25;-0.25;-0.1];
[Aeq_test1,beq_test1]=eq_cons(1,A,B,eY0,npred,nstates,ninputs);

%% 4.4 write and test function (at index 1) to generate limits on inputs
[Lb_test1,Ub_test1]=bound_cons(1,U_ref,npred,input_range,nstates,ninputs);


%% 4.5 simulate controller working

%final trajectory
Y=NaN(3,length(T));

%applied inputs
U=NaN(2,length(T));

%input from QP
u_mpc=NaN(2,length(T));

%error in states (actual-reference)
eY=NaN(3,length(T));

%set random initial condition
Y(:,1)=eY0-Y_ref(:,1);

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

L=(Y(1,:)>=3&Y(1,:)<=4);
max_dist_error=max(sqrt(eY(1,L).^2+eY(2,L).^2))


figure(1)
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
