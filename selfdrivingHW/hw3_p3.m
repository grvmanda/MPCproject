b = 1.5 ; 
L = 3 ;
dt=0.05;

%remember the format for z is as follows:
%z=[x0 y0 th0 x1 y1 th1 ... xn yn thn u0 d0 ... u(n-1) d(n-1)]';
    
%3.1
ub = [repmat([8 3 pi/2]', [121 1]);...
    repmat([1 0.5]', [120 1])];

lb = [repmat([-1 -3 -pi/2]', [121 1]);...
    repmat([0 -0.5]', [120 1])];
nsteps = 121;
%3.4
%%%%%%%%%%%%%%%% no need to change these lines  %%%%%%%%%%%%%%%%%%%%%%
options = optimoptions('fmincon','SpecifyConstraintGradient',true,...
                       'SpecifyObjectiveGradient',true) ;
x0=zeros(1,5*nsteps-2);
cf=@costfun
nc=@nonlcon
z=fmincon(cf,x0,[],[],[],[],lb',ub',nc,options);

Y0=reshape(z(1:3*nsteps),3,nsteps)';
U=reshape(z(3*nsteps+1:end),2,nsteps-1);
u=@(t) [interp1(0:dt:119*dt,U(1,:),t,'previous','extrap');...
        interp1(0:dt:119*dt,U(2,:),t,'previous','extrap')];
[T1,Y1]=ode45(@(t,x) odefun(x,u(t)),[0:dt:120*dt],[0 0 0]);
[T2,Y2]=ode45(@(t,x) odefun(x,u(t)),[0:dt:120*dt],[0 0 -0.01]);
plot(Y0(:,1),Y0(:,2),Y1(:,1),Y1(:,2),Y2(:,1),Y2(:,2))
theta = 0:0.01:2*pi;
hold on
plot((0.7*cos(theta)+3.5),(0.7*sin(theta)-0.5))
hold on
plot(0,0,'x');
legend('fmincon trajectory','ode45 trajectory using x0 = [0;0;0]',...
    'ode45 trajectory using x0 = [0;0;-0.01]','Buffered Obstacle','Start');
ylim([-2,2]);
xlim([-1,8]);
xlabel('x');
ylabel('y');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%3.2
function [g,h,dg,dh]=nonlcon(z)
    % size of g must be 121 x 1 (no.of time steps);
    % size of dg must be 603 x 121 = Transpose(no. of time steps x no. of elements in 'z');
    % size of h must be 363  1 ((no. of time steps * no. of states) x 1)
    % size of dh must be 603 x 363 = Transpose((no. of time steps * no. of states) x no. of elements in 'z') ;
    g = zeros(121, 1);
%     dg = zeros(603, 121);
    dg = zeros(121, 603);   % transpose it later
    h = zeros(363, 1);
%     dh = zeros(603, 363);
    dh = zeros(363, 603);   % transpose it later
    
    x = @(i) z((i)*3+1);
    y = @(i) z((i)*3+2);
    psi = @(i) z((i)*3+3);
    u = @(i) z(363+(2*i+1));
    delta = @(i) z(363+(2*i+2));
    
    xdot = @(i) (u(i)*cos(psi(i)) - 0.5*u(i)*tan(delta(i))*sin(psi(i)));
    ydot = @(i) (u(i)*sin(psi(i)) + 0.5*u(i)*tan(delta(i))*cos(psi(i)));
    psidot = @(i) (u(i)/3)*tan(delta(i));
    
    for i=0:1:120
        g(i+1) = (0.7)^2 - (x(i) - 3.5)^2 - (y(i) + 0.5)^2;
    end
    
    for i = 0:1:120
        dg(i+1, ((i)*3+1):((i)*3+2)) = ...
            [(-2*(x(i)-3.5)), (-2*(y(i)+0.5))];
%             [(-2*(x(i)-3.5)*xdot(i)), (-2*(y(i)+0.5)*ydot(i))];
    end
    
    dg = dg';
    
    h(1:3, 1) = [x(0) y(0) psi(0)];
    
    for i = 1:1:120
        
        h(3*i+1) = x(i) - x(i-1) - 0.05*xdot(i-1);
        h(3*i+2) = y(i) - y(i-1) - 0.05*ydot(i-1);
        h(3*i+3) = psi(i) - psi(i-1) - 0.05*psidot(i-1);
        
    end
    
    dh(1, 1) = 1;
    dh(2, 2) = 1;
    dh(3, 3) = 1;
    
    for i = 1:1:120
        
        dh((3*i+1), (3*i+1)) = 1;                               % x(i)
        dh((3*i+1), (3*(i-1)+1)) = -1;                          % x(i-1)
        dh((3*i+1), (3*(i-1)+3)) = 0.05*u(i-1)*sin(psi(i-1))... % psi(i-1)
            +0.5*0.05*u(i-1)*tan(delta(i-1))*cos(psi(i-1));
        
        dh((3*i+2), (3*i+2)) = 1;                               % y(i)
        dh((3*i+2), (3*(i-1)+2)) = -1;                          % y(i-1)
        dh((3*i+2), (3*(i-1)+3)) = -0.05*(u(i-1)*cos(psi(i-1))... % psi(i-1)
            -0.5*u(i-1)*tan(delta(i-1))*sin(psi(i-1)));
        
        dh((3*i+3), (3*i+3)) = 1;                               % psi(i)
        dh((3*i+3), (3*(i-1)+3)) = -1;                          % psi(i-1)
        
    end
    
    for i = 1:1:120
        dh((3*i+1), (363+2*(i-1)+1)) = -0.05*(cos(psi(i-1))...          % u(i-1)
            -0.5*tan(delta(i-1))*sin(psi(i-1)));
        dh((3*i+1), (363+2*(i-1)+2)) = 0.05*0.5*u(i-1)*...              % delta(i-1)
            (sec(delta(i-1)))^2*sin(psi(i-1));
        
        dh((3*i+2), (363+2*(i-1)+1)) = -0.05*(sin(psi(i-1))...          % u(i-1)
            +0.5*tan(delta(i-1))*cos(psi(i-1)));
        dh((3*i+2), (363+2*(i-1)+2)) = -0.05*0.5*u(i-1)*...             % delta(i-1)
            (sec(delta(i-1)))^2*cos(psi(i-1));
        
        dh((3*i+3), (363+2*(i-1)+1)) = -0.05/3*tan(delta(i-1));         % u(i-1)
        dh((3*i+3), (363+2*(i-1)+2)) = -0.05*u(i-1)/3*(sec(delta(i-1)))^2; %del(i-1)
    end
    
    dh = dh';
    
end

%3.3
function [J, dJ] = costfun(z)
    % size of J must be 1 x 1
    % size of dJ must be 1 x 603 (1 x no. of elements in 'z')
    x = @(i) z((i)*3+1);
    y = @(i) z((i)*3+2);
    psi = @(i) z((i)*3+3);
    u = @(i) z(363+(2*i+1));
    delta = @(i) z(363+(2*i+2));
    
    J = 0;
    for i = 0:120
        J = J + (x(i)-7)^2 + y(i)^2 + psi(i)^2;
    end
    
    for j = 0:119
        J = J + u(j)^2 + delta(j)^2;
    end
    
    dJ = zeros(1, 603);
    
    for i = 0:120
        dJ(3*(i)+1) = 2*(x(i)-7);
        dJ(3*(i)+2) = 2*y(i);
        dJ(3*(i)+3) = 2*psi(i);
    end
    
    for j = 0:119
        dJ(363+2*j+1) = 2*u(j);
        dJ(363+2*j+2) = 2*delta(j);
    end
    
end

function [dx] = odefun(x,u)
    b = 1.5 ; 
    L = 3 ;
    dx = [u(1)*cos(x(3))-(b/L)*u(1)*tan(u(2))*sin(x(3)) ;...
          u(1)*sin(x(3))+(b/L)*u(1)*tan(u(2))*cos(x(3)) ;...
          u(1)*tan(u(2))/L] ;
end