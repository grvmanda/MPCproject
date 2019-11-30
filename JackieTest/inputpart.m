function [ip] = inputpart(x,u)
    b = 1.5 ; 
    L = 3 ;
    ip = [cos(x(3))-(b/L)*tan(u(2))*sin(x(3)) -(b*u(1)*sin(x(3)))/(L*cos(u(2))^2) ;...
          sin(x(3))+(b/L)*tan(u(2))*cos(x(3))  (b*u(1)*cos(x(3)))/(L*cos(u(2))^2) ;...
          tan(u(2))/L                          u(1)/(L*cos(u(2))^2)] ;
end