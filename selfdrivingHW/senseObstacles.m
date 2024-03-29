function Xobs_seen = senseObstacles(curr_pos, Xobs)
% Xobs_seen = senseObstacles(curr_pos, Xobs)
% 
% Given the current vehicle position, sense the obstacles within 150m.
% 
% INPUTS:
%   curr_pos   a 2-by-1 vector where the 1st and 2nd elements represent the
%              x and y coordinates of the current vehicle position
%   
%   Xobs       a cell array generated by generateRandomObstacles.m
%   
% OUTPUTS:
%   Xobs_seen  a cell array which contains all obstacles that are no 
%              greater than 150m from the vehicle. Each cell has the same
%              structure as the cells in Xobs.
%   
% Written by: Jinsun Liu
% Created: 31 Oct 2019



    if (~isempty(Xobs))
        Xobs_mat = cell2mat(Xobs');
        dist = (Xobs_mat(:,1) - curr_pos(1)).^2 + (Xobs_mat(:,2) - curr_pos(2)).^2;
        idx = unique(ceil(find(dist<=10^2)/4));
        Xobs_seen = {Xobs{idx}};
    else
        Xobs_seen = {};
    end
end