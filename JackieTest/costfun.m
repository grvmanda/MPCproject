function [J, dJ] = costfun(z)
    if size(z,2) > size(z,1)
        z = z' ;
    end
    nsteps = (size(z,1)+2)/5 ;

    zx = z(1:3*nsteps) ;
    zu = z(3*nsteps+1:end) ;
    R=eye(2*nsteps-2);

    nom=zeros(3*nsteps,1) ;
    nom(1:3:3*nsteps) = 7 ;
    Q=eye(3*nsteps);

    J = zu'*R*zu+(zx-nom)'*Q*(zx-nom) ;
    dJ = [2*Q*zx-2*Q*nom;...
          2*R*zu]' ;
end