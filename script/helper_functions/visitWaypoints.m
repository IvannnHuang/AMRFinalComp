function [v, w, gotopt] = visitWaypoints(waypoints, gotopt, closeEnough, pose, epsilon)
% visitWaypoints: calculate desired forward and angular velocity to drive
% the robot along a series of waypoints. 
% Inputs:
%  waypoints: a n-by-2 matrix where each row is the (x, y) coordinate of a waypoint. 
%  gotopt: the index of the waypoint being driven toward
%  closeEnough: the waypoint is considered as reached if the robot is
%   within closeEnough of the waypoint location. 
%  pose: current pose of the robot. (x, y, theta)
%  epsilon: turn radius
% Output:
%  v: desired forward velocity
%  w: desired angular velocity
if gotopt > size(waypoints, 1)
    v = 0;
    w = 0;
else
    px = pose(1);
    py = pose(2);
    theta = pose(3);
    goalX = waypoints(gotopt, 1);
    goalY = waypoints(gotopt, 2);
    vx = goalX - px;
    vy = goalY - py;
    if sqrt(vx^2 + vy^2) > closeEnough
        [v, w] = feedbackLin(vx, vy, theta, epsilon);
    else
        v = 0;
        w = 0;
        gotopt = gotopt + 1;
    end
end
end