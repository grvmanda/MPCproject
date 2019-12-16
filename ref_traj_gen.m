function [poses, delta_ref] = ref_traj_gen(startPose, goalPose, minRadius) 

    % Path is a semi-circle for now.
    % r1 is radius of inner circle
    % r2 is radius of outer circle

    dubConnObj = dubinsConnection;

    pathSegObj = connect(dubConnObj,startPose,goalPose);

    dubConnObj.MinTurningRadius = minRadius;

    [pathSegObj, pathCosts] = connect(dubConnObj,startPose,goalPose);
    pathSegObj{1}.MotionTypes

    figure;
    show(pathSegObj{1})
    
    len = pathSegObj{1}.Length;

    constVel = 5;
    dt = 0.01;
    dlen = constVel*dt;

    poses = interpolate(pathSegObj{1},0:dlen:len);
%     figure;
%     quiver(poses(:,1),poses(:,2),cos(poses(:,3)),sin(poses(:,3)),0.5)

    % delta reference calculation
    % TODO: add something here for directions
    delta_ref = ones(1, length(poses))*0.02635; % delta_ref to follow a curve of constant radius  
    
end