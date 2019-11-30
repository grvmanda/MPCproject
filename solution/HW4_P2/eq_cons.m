function [ Aeq, beq ] = eq_cons(initial_idx,A,B,x_initial,npred,nstates,ninputs)
%size of decision variable and size of part holding states
zsize = (npred+1)*nstates+npred*ninputs ;
xsize = (npred+1)*nstates ;

Aeq = zeros(xsize,zsize) ;
Aeq(1:nstates,1:nstates) = eye(nstates) ; %initial condition 
beq = zeros(xsize,1) ;
beq(1:nstates) = x_initial ;

% create index vectors
state_idxs = nstates+1:nstates:xsize ;
input_idxs = xsize+1:ninputs:zsize ;

for i = 1:npred
    %negative identity for i+1
    Aeq(state_idxs(i):state_idxs(i)+nstates-1,state_idxs(i):state_idxs(i)+nstates-1) = -eye(nstates) ;
    
    %A matrix for i
    Aeq(state_idxs(i):state_idxs(i)+nstates-1,state_idxs(i)-nstates:state_idxs(i)-1) = A(initial_idx+i-1) ;
    
    %B matrix for i
    Aeq(state_idxs(i):state_idxs(i)+nstates-1,input_idxs(i):input_idxs(i)+ninputs-1) = B(initial_idx+i-1) ;
end

end