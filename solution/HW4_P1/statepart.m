function [pd] = statepart(x,u)
    b = 1.5 ; 
    L = 3 ;
    pd = zeros(3,3) ;
    pd(:,3) = [-u(1)*sin(x(3))-(b/L)*u(1)*tan(u(2))*cos(x(3)) ;...
                u(1)*cos(x(3))-(b/L)*u(1)*tan(u(2))*sin(x(3)) ;...
                0] ;
end