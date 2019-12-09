function [poses, delta_ref] = ref_traj_gen(startPose, goalPose) 

    % Path is a semi-circle for now.
    % r1 is radius of inner circle
    % r2 is radius of outer circle

    dubConnObj = dubinsConnection;

    pathSegObj = connect(dubConnObj,startPose,goalPose);

    dubConnObj.MinTurningRadius = 105;

    [pathSegObj, pathCosts] = connect(dubConnObj,startPose,goalPose);
    pathSegObj{1}.MotionTypes

    len = pathSegObj{1}.Length;

    constVel = 5;
    dt = 0.01;
    dlen = constVel*dt;

    poses = interpolate(pathSegObj{1},0:dlen:len);

    % delta reference calculation
    % TODO: add something here for directions
    delta_ref = ones(1, length(poses))*0.02635; % delta_ref to follow a curve of constant radius  
    
end