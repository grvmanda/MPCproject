function [g,h,dg,dh]=nonlcon(z)
    if size(z,2) > size(z,1)
        z = z' ;
    end
    nsteps = (size(z,1)+2)/5 ;
    b = 1.5 ; 
    L = 3 ;
    dt = 0.05 ;

    zx = z(1:nsteps*3) ;
    zu = z(nsteps*3+1:end) ;

    g = zeros(nsteps,1) ;
    dg = zeros(nsteps,5*nsteps-2) ;

    h = zeros(3*nsteps,1) ;
    dh = zeros(3*nsteps,5*nsteps-2);

    h(1:3) = z(1:3,:) ;
    dh(1:3,1:3) = eye(3) ;

    for i = 1:nsteps
        if i == 1
            h(1:3) = z(1:3,:) ;
            dh(1:3,1:3) = eye(3) ; 
        else
            h(3*i-2:3*i) = zx(3*i-2:3*i)-zx(3*i-5:3*i-3)-...
                               dt*odefun(zx(3*i-5:3*i-3),zu(2*i-3:2*i-2)) ;
            dh(3*i-2:3*i,3*i-5:3*i) = [-eye(3)-dt*statepart(zx(3*i-5:3*i-3),zu(2*i-3:2*i-2)),eye(3)] ;
            dh(3*i-2:3*i,3*nsteps+2*i-3:3*nsteps+2*i-2) = -dt*inputpart(zx(3*i-5:3*i-3),zu(2*i-3:2*i-2)) ;
        end
        g(i) = (0.7^2)-((z(3*i-2)-3.5)^2+(z(3*i-1)+0.5)^2) ;
        dg(i,3*i-2:3*i-1) = -2*[z(3*i-2)-3.5, z(3*i-1)+0.5] ;
    end

    dg = dg' ;
    dh = dh' ;
end