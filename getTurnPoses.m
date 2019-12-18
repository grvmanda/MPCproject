function [poses,nextStartPose,nextGoalPose] = getTurnPoses(turn,Yt)
    
    minTurnRadius = 5;
    dist_travelled = 10;
    leftLaneCenterR = 98.5;
    rightLaneCenterR = 101.5;
    laneCenterR = leftLaneCenterR;
    
    if turn == 1
        laneCenterR = rightLaneCenterR;
    end
    
    startPose = Yt(:,end)';
    theta_diff = dist_travelled/laneCenterR;
    new_theta = Yt(3,end)+theta_diff;
    nextStartPose = [laneCenterR*sin(new_theta),...
                -laneCenterR*cos(new_theta),...
                new_theta];
            
    [poses,~] = ref_traj_gen(startPose, nextStartPose, minTurnRadius);
    nextGoalPose = [0 laneCenterR pi];
end

