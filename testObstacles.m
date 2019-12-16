clear variables; close all; clc

r1 = 97;
r2 = 103;

leftLaneCenterR = 98.5;
rightLaneCenterR = 101.5;

path.numPoints = 1001;
thetaVal = linspace(-pi/2, pi/2, path.numPoints);

path.xL = r1*cos(thetaVal);
path.xR = r2*cos(thetaVal);
path.yL = r1*sin(thetaVal);
path.yR = r2*sin(thetaVal);
path.xCL = ((r1 + r2)/2)*cos(thetaVal);
path.yCL = ((r1 + r2)/2)*sin(thetaVal);
path.theta = thetaVal;
path.cline = [path.xCL;...
    path.yCL];

Nobs = 20;
[Xobs, obs_heading] = generateRandomObstacles(Nobs,path);

figure;
hold on
plot(path.xL, path.yL, 'k', path.xR, path.yR, 'k', ...
    path.xCL, path.yCL, 'b');

for i=1:Nobs
    ob = Xobs{i};
    plot(ob(:,1), ob(:,2));
end