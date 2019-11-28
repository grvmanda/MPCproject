function [J, dJ] = costfun(z, numSteps, xFinal, yFinal, psiFinal)

    % size of J must be 1 x 1
    x = @(i) z((i)*3+1);
    y = @(i) z((i)*3+2);
    psi = @(i) z((i)*3+3);
    u = @(i) z((numSteps)*3+(2*i+1));
    delta = @(i) z((numSteps)*3+(2*i+2));

    J = 0;
    for i = 0:numSteps - 1
        J = J + (x(i)-xFinal)^2 + (y(i)-yFinal)^2 + (psi(i)-psiFinal)^2;
    end

    for j = 0:numSteps - 2
        J = J + u(j)^2 + delta(j)^2;
    end
    
    dJ = zeros(1, 3*(numSteps)+2*(numSteps - 1));

    for i = 0:numSteps - 1
        dJ(3*(i)+1) = 2*(x(i)-xFinal);
        dJ(3*(i)+2) = 2*(y(i)-yFinal);
        dJ(3*(i)+3) = 2*(psi(i)-psiFinal);
    end

    for j = 0:numSteps - 2
        dJ(3*(numSteps)+2*j+1) = 2*u(j);
        dJ(3*(numSteps)+2*j+2) = 2*delta(j);
    end

end