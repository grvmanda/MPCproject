% parameters
L = 3 ;
b = 1.5 ;

% number of states and inputs in dynamic model
nstates = 3 ;
ninputs = 2 ;

% input ranges
input_range = [   0,   1;...
               -0.5, 0.5] ;

% time discretization
dt = 0.01 ;

% time span
T = 0:dt:6 ;

% load reference trajectory
load('part1_traj_05_timestep.mat')
U_ref = interp1(0:0.05:6,[U,U(:,end)]',T)' ;
Y_ref = interp1(0:0.05:6,Y,T)' ;

% discrete-time A and B matrices
A = @(i) eye(3)+dt*[0, 0, -U_ref(1,i)*sin(Y_ref(3,i))-(U_ref(1,i)*b/L)*cos(Y_ref(3,i))*tan(U_ref(2,i)) ;...
                    0, 0,  U_ref(1,i)*cos(Y_ref(3,i))-(U_ref(1,i)*b/L)*sin(Y_ref(3,i))*tan(U_ref(2,i)) ;...
                    0, 0,                                                                            0 ] ;
                
B = @(i) dt*[cos(Y_ref(3,i))-b/L*sin(Y_ref(3,i))*tan(U_ref(2,i)), -(U_ref(1,i)*b*sin(Y_ref(3,i)))/(L*cos(U_ref(2,i))^2) ;...
             sin(Y_ref(3,i))+b/L*cos(Y_ref(3,i))*tan(U_ref(2,i)),  (U_ref(1,i)*b*cos(Y_ref(3,i)))/(L*cos(U_ref(2,i))^2) ;...
                                             1/L*tan(U_ref(2,i)),                      U_ref(1,i)/(L*cos(U_ref(2,i))^2) ] ;
                                         
% number of decision variables
npred = 10 ;
Ndec = (npred+1)*nstates+ninputs*npred ;

% equality constraint
eY0 = [0.25;-0.25;-0.1] ;
[Aeq_test1,beq_test1] = eq_cons(1,A,B,eY0,npred,nstates,ninputs) ;

% limits on inputs
[Lb_test1,Ub_test1] = bound_cons(1,U_ref,npred,input_range,nstates,ninputs) ;

% simulation variables
Y = NaN(3,length(T)) ;
U = NaN(2,length(T)) ;
u_mpc = NaN(2,length(T)) ;
eY = NaN(3,length(T)) ;
Y(:,1) = eY0-Y_ref(:,1) ;
Q = [1,1,0.5] ;
R = [0.1,0.01] ;
opts = optimoptions('quadprog','Display','off') ;

for i = 1:length(T)-1
    % shorten prediction horizon if we are at the end of trajectory
    npred_i = min([npred,length(T)-i]) ;
    
    % calculate error
    eY(:,i) = Y(:,i)-Y_ref(:,i) ;

    % generate constraints
    [Aeq,beq] = eq_cons(i,A,B,eY(:,i),npred_i,nstates,ninputs) ;
    [Lb,Ub] = bound_cons(i,U_ref,npred_i,input_range,nstates,ninputs) ;
    
    % cost function matrices
    H = diag([repmat(Q,[1,npred_i+1]),repmat(R,[1,npred_i])]) ;
    f = zeros(nstates*(npred_i+1)+ninputs*npred_i,1) ;
    
    % run MPC
    [x,fval] = quadprog(H,f,[],[],Aeq,beq,Lb,Ub,[],opts) ;
    
    % get linearized input
    u_mpc(:,i) = x(nstates*(npred_i+1)+1:nstates*(npred_i+1)+ninputs) ;
    
    % get input
    U(:,i) = u_mpc(:,i)+U_ref(:,i) ;
    
    % simulate model
    [~,ztemp] = ode45(@(t,z)kinematic_bike_dynamics(t,z,U(:,i),0,b,L),[0 dt],Y(:,i)) ;
    
    % store final state
    Y(:,i+1) = ztemp(end,:)' ;
end

% find max distance error
L = (Y(1,:) >= 3 & Y(1,:) <= 4) ;
max_dist_error = max(sqrt(eY(1,L).^2+eY(2,L).^2)) ;

% plot results
figure(1)

subplot(3,1,1)
plot(Y_ref(1,:),Y_ref(2,:),Y(1,:),Y(2,:))
xlabel('x [m]') ; ylabel('y [m]') ;

subplot(3,1,2)
plot(Y_ref(1,:),U_ref(1,:),Y_ref(1,:),U(1,:))
xlabel('x [m]') ; ylabel('u [m/s]') ;

subplot(3,1,3)
plot(Y_ref(1,:),U_ref(2,:),Y_ref(1,:),U(2,:))
xlabel('x [m]') ; ylabel('\delta_f [rad]') ;