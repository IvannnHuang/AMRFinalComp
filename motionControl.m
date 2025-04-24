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

%mapFile = 'compMap.mat';
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

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 60;
end

%fails when not connected to a physical robot 
try 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'deadReck', [], ...
                   'path', [], ...
                   'nodes', []);

% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

tic
%collect initialization data by spinning around
numPoints = 12;
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
    [mu_t, ~, theta_t] = initializeStartPos(start(1:2)', sigma_0, dataStore.rsdepth(6:8,3:11), map);
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

%build path of robot
for i = 1:size(waypoints,1)
    goal = waypoints(i,:);
    [path, nodes] = buildRRT(mapFile, boundary, start(1:2), goal, radius);
    dataStore.path = [dataStore.path; path];
    dataStore.nodes = [dataStore.nodes; nodes];
    %set start to previous goal
    start = goal;
end
for i = 1:size(wallCenters,1)
    goal = wallCenters(i,:);
    [path, nodes] = buildRRT(mapFile, boundary, start, goal, radius);
    dataStore.path = [dataStore.path; path];
    dataStore.nodes = [dataStore.nodes; nodes];
    %set start to previous goal
    start = goal;
end
for i = 1:size(ECwaypoints,1)
    goal = ECwaypoints(i,:);
    [path, nodes] = buildRRT(mapFile, boundary, start, goal, radius);
    dataStore.path = [dataStore.path; path];
    dataStore.nodes = [dataStore.nodes; nodes];
    %set start to previous goal
    start = goal;
end

figure;
hold on;
plot(dataStore.path(:,1), dataStore.path(:,2));
for i = 1:length(map)
    plot([map(i,1),map(i,3)],[map(i,2),map(i,4)], 'k');
end
hold off;

SetFwdVelAngVelCreate(Robot, 0,0 );

end
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Kaplan, Natalie

v_L = fwdVel - angVel * wheel2Center;
v_R = fwdVel + angVel * wheel2Center;

maxWheelVel = max(abs(v_L), abs(v_R));
scaleFactor = 1;

if maxWheelVel > maxV
    scaleFactor = maxV/maxWheelVel;
end

cmdV = .5*(v_L + v_R)*scaleFactor;
cmdW = (1/(wheel2Center*2))*(v_R-v_L)*scaleFactor;

end