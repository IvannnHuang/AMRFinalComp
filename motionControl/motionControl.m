function[dataStore] = motionControl(Robot, maxTime)
% rrtPlanner: build an RRT and once a collision-free path has been found, 
% have the robot follow the path
% 
%   INPUTS
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Final Project
%   Kaplan, Natalie

addpath("maps\");
addpath("plotting\");
addpath("helper_functions\");

mapFile = 'map1_3credits.mat';
map = load(mapFile).map;
optWalls = load(mapFile).optWalls;
waypoints = sortrows(load(mapFile).waypoints, 'ascend');
ECwaypoints = sortrows(load(mapFile).ECwaypoints, 'descend');
stayAwayPoints = load(mapFile).stayAwayPoints;
beaconLoc = load(mapFile).beaconLoc;
radius = 0.2;
boundary = [min(map(1:4,1)),min(map(1:4,2)),max(map(1:4,1)),max(map(1:4,2))];

%locations of center points of optional walls
wallCenters = ones(size(optWalls,1),2);
for i = 1:size(optWalls,1)
    wallCenters(i,:) = [(optWalls(i,3)-optWalls(i,1))/2, (optWalls(i,4)-optWalls(i,2))/2];
end
wallCenters = sortrows(wallCenters, 'ascend');



if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 60;
end
try 
    CreatePort=Robot.CreatePort;
catch
    CreatePort = Robot;
end
global dataStore;
dataStore = struct('truthPose', [],'odometry', [],'rsdepth', [],'bump', [], ...
                   'beacon', [],'deadReck', [],'path', [],'nodes', []);
noRobotCount = 0;
tic


%collect initialization data by spinning around
numPoints = 18;
for i = 1:numPoints
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    turnAngle(Robot, radius, 360/numPoints);
end

start = [0,0]; %initial pose estimate is the middle
%if a beacon was detected, use that to initialize position
if ~isempty(dataStore.beacon)
    start = posFromBeacon(dataStore.beacon(:,2:end), beaconLoc);
    disp('Beacon was used')
%if there is no beacon, use the Kalman filter to get a better position
else
    sigma_0 = [100 0;0 100]; 
    [mu_t, ~, theta_t] = initializeStartPos(start(1:2)', sigma_0, dataStore.rsdepth(:,2:11), map);
    start = [mu_t; theta_t]';
    disp('KF used')
end

figure;
hold on;
plot(start(1), start(2), 'go');

%snap to closest waypoint
closestPt = 0;
closestDist = 10;
for i = 1:size(waypoints,1)
    dist = norm([waypoints(i,1)-start(1), waypoints(i,2)-start(2)]);
    if dist < closestDist
        closestDist = dist;
        closestPt = i;
    end
end
start = waypoints(closestPt,:);

plot(start(1), start(2), 'g*');
for i = 1:length(map)
    plot([map(i,1),map(i,3)],[map(i,2),map(i,4)], 'k');
end
hold off;

SetFwdVelAngVelCreate(Robot, 0,0 );

end