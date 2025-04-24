% DEPTHPREDICT: predict the depth measurements for a robot given its pose
% and the map
%
%   DEPTH = DEPTHPREDICT(ROBOTPOSE,MAP,SENSORORIGIN,ANGLES) returns
%   the expected depth measurements for a robot 
%
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2
%   Huang, Ivan

function[depth] = depthPredict(robotPose,map,sensorOrigin,angles)

    maxRange = 100;
    numAngles = length(angles);
    mapSize = size(map, 1);

    % convert wall to sensor frame
    map_sensor = zeros(mapSize, 4);
    for i = 1: mapSize
        wall_1 = [cos(robotPose(3)) -sin(robotPose(3)) robotPose(1); sin(robotPose(3)) cos(robotPose(3)) robotPose(2); 0 0 1 ]^-1*[map(i, 1); map(i, 2); 1];
        x1 = wall_1(1) - sensorOrigin(1);
        y1 = wall_1(2) - sensorOrigin(2);
        wall_2 = [cos(robotPose(3)) -sin(robotPose(3)) robotPose(1); sin(robotPose(3)) cos(robotPose(3)) robotPose(2); 0 0 1 ]^-1*[map(i, 3); map(i, 4); 1];
        x2 = wall_2(1) - sensorOrigin(1);
        y2 = wall_2(2) - sensorOrigin(2);
        map_sensor(i, :) = [x1 y1 x2 y2];
    end

    % depth detection
    depth = zeros(numAngles, 1);
    intersects = zeros(numAngles, mapSize);
    rays = zeros(numAngles, 4);
    for i = 1:numAngles
        rays(i, 1:2) = [0 0];
        rays(i, 3:4) = [maxRange*cos(angles(i)) maxRange*sin(angles(i))];
        
        % Check intersection with all walls
        for j = 1:mapSize
            [isect, x, y] = intersectPoint(rays(i, 1), rays(i, 2), rays(i, 3), rays(i, 4), ...
                                           map_sensor(j, 1), map_sensor(j, 2), map_sensor(j, 3), map_sensor(j, 4));
            if (size(x) == 0)
                intersects(i, j) = maxRange;   
            else
                intersects(i, j) = x;
            end
        end
        depth(i) = min(intersects(i, :));
    end
end
