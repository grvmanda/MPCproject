% input ranges (first row is longitudinal velocity, second row is wheel
% angle)
% Should be of the form [0, 5; -0.5, 0.5]

% npred is the length of the prediction horizon

function [Y,U] = runMPC(input_range,npred,T_length,Y_ref,U_ref,A,B)
    
    % Define vehicle parameters
    b = 1.45;
    L = 2.8;

    % Define number of states and inputs
    nstates = 3;
    ninputs = 2;
    
    % Define simulation time step
    dt = 0.01;
    
    % Preallocate space
    Y = NaN(nstates,T_length); % final trajectory
    U = NaN(ninputs,T_length); % applied inputs
    
    u_mpc = NaN(size(U)); % input from QP
    eY = NaN(size(Y)); % error in states (actual - reference)
    
    % Set random initial condition
    eY0 = [0;-0.25;0]; % parameterize?
    Y(:,1) = Y_ref(:,1) - eY0;
    
    
    for i = 1:T_length - 1
        % shorten prediction horizon if we are at the end of trajectory
        npred_i = min([npred,T_length-i]);

        % calculate error
        eY(:,i) = Y(:,i) - Y_ref(:,i);

        % generate equality constraints
        [Aeq,beq] = eq_cons(i,A,B,eY(:,i),npred_i,nstates,ninputs);

        % generate boundary constraints
        [Lb,Ub] = bound_cons(i,U_ref,npred_i,input_range,nstates,ninputs);

        %cost for states
        Q = [1,1,0.5];

        % cost for inputs
        R = [0.1,0.01];

        H = diag([repmat(Q,[1,npred_i+1]),repmat(R,[1,npred_i])]);

        f = zeros(nstates*(npred_i+1)+ninputs*npred_i,1);

        [x,fval] = quadprog(H,f,[],[],Aeq,beq,Lb,Ub);

        % get linearized input
        u_mpc(:,i) = x(nstates*(npred_i+1)+1:nstates*(npred_i+1)+ninputs);

        % get input
        U(:,i) = u_mpc(:,i) + U_ref(:,i);

        % simulate model
        [~,ztemp] = ode45(@(t,z)kinematic_bike_dynamics(t,z,U(:,i),0,b,L),[0 dt],Y(:,i));

        % store final state
        Y(:,i+1) = ztemp(end,:)';
        
        % check for obstacles ahead
        hasObs = ~isempty(senseObstacles(Y(:,i+1),Xobs)); % might need to fix
        
        if hasObs
            U = U(:,1:i);
            Y = Y(:,1:i+1);
            break;
        end
    end
end