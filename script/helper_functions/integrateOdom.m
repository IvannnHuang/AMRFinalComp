% integrateOdom: Calculate the robot pose in the initial frame based on the
% odometry
% 
% [finalPose] = integrateOdom(initPose,dis,phi) returns a 3-by-N matrix of the
% robot pose in the initial frame, consisting of x, y, and theta.

%   INPUTS
%       initPose    robot's initial pose [x y theta]  (3-by-1)
%       d     distance vectors returned by DistanceSensorRoomba (1-by-N)
%       phi     angle vectors returned by AngleSensorRoomba (1-by-N)

% 
%   OUTPUTS
%       finalPose     The final pose of the robot in the initial frame
%       (3-by-N)

%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
%   Huang, Ivan

function finalPose = integrateOdom(initPose, d, phi)
% Single-step motion update for one particle
% initPose: [x; y; theta]
% d: distance traveled (m)
% phi: rotation (rad)

    % Add noise to motion
    % d = d*(1+0.05*randn(1, 1));
    % phi = phi*(1+0.1*randn(1, 1)); 
    d = d*(1+0.02*randn(1, 1));
    phi = phi*(1.30+0.03*randn(1, 1)); 


    x = initPose(1);
    y = initPose(2);
    theta = initPose(3);

    if abs(phi) < 1e-6
        x = x + d * cos(theta);
        y = y + d * sin(theta);
    else
        R = d / phi;
        x = x + R * (sin(theta + phi) - sin(theta));
        y = y - R * (cos(theta + phi) - cos(theta));
    end
    theta = theta + phi;
    theta = mod(theta + pi, 2*pi) - pi;

    finalPose = [x; y; theta];
end
