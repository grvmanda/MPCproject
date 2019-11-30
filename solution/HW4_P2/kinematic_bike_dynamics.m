function dzdt = kinematic_bike_dynamics(t,z,U_in,T,b,L)

if length(T)<=1 || isempty(T) || size(U_in,2)==1
    delta = U_in(2) ;
    u = U_in(1) ;
else
    delta = interp1(T',U_in(2,:)',t,'previous') ;
    u = interp1(T',U_in(1,:)',t,'previous') ;
end

dzdt = [u*cos(z(3))-b/L*u*tan(delta)*sin(z(3)) ;...
        u*sin(z(3))+b/L*u*tan(delta)*cos(z(3)) ;...
                                u/L*tan(delta) ] ;
end