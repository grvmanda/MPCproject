clear variables; close all; clc

% Define quadratic cost function


% Define bounds to be active only on left half of circle
% 0 <= x1 <= 50;
% 0 <= x2 <= 100;
lb = [0, 0];
ub = [50, 100];

nonlcon = @mynonlcon;

function [c, ceq] = mynonlcon(x)
    % Define inequality constraints, no equality for now
    % c1: inside bigger circle of radius 50 centered at (50, 50)
    % c2: outside smaller circle of radius 30 centered at (50, 50)
    c1 = (x(1) - 50)^2 + (x(2) - 50)^2 - 50^2;
    c2 = 30^2 - (x(1) - 50)^2 - (x(2) - 50)^2;

    c = [c1; c2]; % nonlinear inequality constraints
    ceq = []; % nonlinear equality constraints
end