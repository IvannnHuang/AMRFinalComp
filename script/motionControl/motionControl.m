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
ECwaypoints = sortrows(load(mapFile).ECwaypoints, 'ascend');
stayAwayPoints = load(mapFile).stayAwayPoints;
beaconLoc = load(mapFile).beaconLoc;
radius = 0.2;
boundary = [min(map(1:4,1)),min(map(1:4,2)),max(map(1:4,1)),max(map(1:4,2))];

%locations of center points of optional walls
wallCenters = ones(size(optWalls,1),2);
for i = 1:size(optWalls,1)
    wallCenters(i,:) = [(optWalls(i,3)-optWalls(i,1))/2, (optWalls(i,4)-optWalls(i,2))/2];
    wallCenters(i,:) = wallCenters(i,:) + [min(optWalls(i,3),optWalls(i,1)), min(optWalls(i,4),optWalls(i,2))];
end
wallCenters = sortrows(wallCenters, 'descend');

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 360;
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
    [mu_t, ~, theta_t] = initializeStartPos(start(1:2)', sigma_0, dataStore.rsdepth(:,3:11), map);
    start = [mu_t; theta_t]';
    disp('KF used')
end

%force position to be inside map
if start(1) > boundary(3)
    start(1) = boundary(3);
elseif start(1) < boundary(1)
    start(1) = boundary(3);
end
if start(2) > boundary(4)
    start(2) = boundary(4);
elseif start(2) < boundary(2)
    start(2) = boundary(2);
end

% figure;
% hold on;
% plot(start(1), start(2), 'go');

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

% plot(start(1), start(2), 'g*');
% for i = 1:length(map)
%     plot([map(i,1),map(i,3)],[map(i,2),map(i,4)], 'k');
% end
% hold off;

%for testing purposes, use truthPose for initialization
% [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
% start = dataStore.truthPose(1,2:4);
% dataStore.deadReck = start;

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

% figure;
% hold on;
% plot(dataStore.path(:,1), dataStore.path(:,2));
% for i = 1:length(map)
%     plot([map(i,1),map(i,3)],[map(i,2),map(i,4)], 'k');
% end
% hold off;

closeEnough = 0.05;
gotopt = 1;

SetFwdVelAngVelCreate(Robot, 0,0);
while toc < maxTime
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    dataStore.deadReck = [dataStore.deadReck; integrateOdom(dataStore.deadReck(end, :)',dataStore.odometry(end, 2),dataStore.odometry(end, 3))'];

    %Set forward velocity
    [cmdV, cmdW] = feedbackLin(dataStore.path(gotopt,1)-dataStore.truthPose(end,2),...
        dataStore.path(gotopt,2)-dataStore.truthPose(end,3), dataStore.truthPose(end,4),0.2);
    [cmdV, cmdW] = limitCmds(cmdV,cmdW,0.2,0.1);

    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end

    %if robot is close enough to point, go to the next one
    if sqrt((dataStore.truthPose(end,2)-dataStore.path(gotopt,1))^2+...
            (dataStore.truthPose(end,3)-dataStore.path(gotopt,2))^2) <= closeEnough
        gotopt=gotopt+1;
        if gotopt > length(dataStore.path)
            SetFwdVelAngVelCreate(Robot, 0,0 );
            break
        end
    end

    pause(0.005);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0 );

end

function [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore)
% This function tries to read all the sensor information from the Create
% and store it in a data structure
%
%   [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore) runs 
% 
%   INPUTS
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       noRobotCount Number of consecutive times the robot was "lost" by the overhead localization
%       dataStore    struct containing logged data
% 
%   OUTPUTS
%       noRobotCount   Updated number of consecutive times the robot was "lost" by the overhead localization
%       dataStore   Updated struct containing logged data
%
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%
%	Modified: Liran 2023


try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
    DistPort=Robot.DistPort;
    TagPort=Robot.TagPort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort=Robot;
    DistPort=Robot;
    TagPort=Robot;
end

    % read truth pose (from overhead localization system)
    try
        try
            [px, py, pt] = OverheadLocalizationCreate(Robot);
            poseX = px; poseY = py; poseTheta = pt;
            dataStore.truthPose = [dataStore.truthPose ; ...
                               toc poseX poseY poseTheta];
            noRobotCount = 0;
        catch
            disp('Overhead localization lost the robot!')
            noRobotCount = noRobotCount + 1;
        end
    catch
        disp('Error retrieving or saving overhead localization data.');
    end
    
    %read odometry distance & angle
    try
        deltaD = DistanceSensorRoomba(CreatePort);
        deltaA = AngleSensorRoomba(CreatePort);
        dataStore.odometry = [dataStore.odometry ; ...
                              toc deltaD deltaA];
    catch
        disp('Error retrieving or saving odometry data.');
    end
 
    
    % read bump data
    try
        [BumpRight BumpLeft DropRight DropLeft DropCaster BumpFront] = ...
            BumpsWheelDropsSensorsRoomba(CreatePort);
        dataStore.bump = [dataStore.bump ; toc ...
            BumpRight BumpLeft DropRight DropLeft DropCaster BumpFront];
    catch
        disp('Error retrieving or saving bump sensor data.');
    end
    


    %read Real Sense depth data
    try
           
        depth_array = RealSenseDist(Robot);
        dataStore.rsdepth = [dataStore.rsdepth ; toc depth_array'];
    catch
        disp('Error retrieving or saving RealSense depth data.');
    end

    %read camera data (beacons)
    try
        tags = RealSenseTag(Robot);
        if ~isempty(tags)
            dataStore.beacon = [dataStore.beacon ; repmat(toc,size(tags,1),1) tags];
        end
    catch
        disp('Error retrieving or saving beacon (AprilTag) data.');
    end
end

function [cmdV, cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon)
% FEEDBACKLIN Transforms Vx and Vy commands into V and omega commands using
% feedback linearization techniques
% Inputs:
%  cmdVx: input velocity in x direction wrt inertial frame
%  cmdVy: input velocity in y direction wrt inertial frame
%  theta: orientation of the robot
%  epsilon: turn radius
% Outputs:
%  cmdV: fwd velocity
%  cmdW: angular velocity
%
% Kaplan, Natalie

velocity = [1,0;0,(1/epsilon)]*[cos(theta),sin(theta);-sin(theta),cos(theta)]*[cmdVx;cmdVy];

cmdV = velocity(1,1);
cmdW = velocity(2,1);
end

function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
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

function [finalPose] = integrateOdom(initPose,d,phi)
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

finalPose=zeros(3,length(phi));

prevX=initPose(1,1);
prevY=initPose(2,1);
prevAngle=initPose(3,1);

for i=1:length(phi)
    finalPose(1,i)=d(1,i)*cos((phi(1,i)/2)+prevAngle)+prevX;
    finalPose(2,i)=d(1,i)*sin((phi(1,i)/2)+prevAngle)+prevY;
    finalPose(3,i)=phi(1,i)+prevAngle;
    prevX=finalPose(1,i);
    prevY=finalPose(2,i);
    prevAngle=finalPose(3,i);
end
end