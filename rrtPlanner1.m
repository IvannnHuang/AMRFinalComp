function[dataStore] = rrtPlanner1(Robot,maxTime)
% rrtPlanner
% 
%   dataStore = TURNINPLACE(Robot,maxTime) runs 
% 
%   INPUTStype
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%   Homework #6
%   Chen, Runbin

defaultRuntime = 180;

addpath("maps\");
addpath("plotting\");
addpath("helper_functions\");

map = load('map1_3credits.mat').map;
mapBoundary = calcMapBoundary(map);
robotRadius = 0.2;
waypoints = load('map1_3credits.mat').waypoints;
goal = waypoints(1, :);
[px, py, ~] = OverheadLocalizationCreate(Robot);
start = [px, py];
[V, E] = RRTwalls(map, mapBoundary, start, goal, robotRadius);
waypoints = findPath([], V, E, start, goal);
gotopt = 1;
epsilon = 0.2;
closeEnough = 0.1;

fig1 = figure(1);
[vertex_plot, edge_plot] = plotRoadmap(V, E, fig1);
hold on
for i = 1:size(map, 1)
    if ~all(isnan(map(i,:))) && ~all(map(i,:) == 0)
        walls_plot = plot([map(i, 1), map(i, 3)], [map(i, 2), map(i, 4)], 'k', 'LineWidth', 1);
    end
end

axis equal;
grid on;

traj_Plot = plot(nan, nan, 'c-', 'LineWidth', 1.5);
goal_plot = scatter(goal(1), goal(2), 'gx', 'LineWidth', 2);
start_plot = scatter(start(1), start(2), 'rx', 'LineWidth', 2);

xlabel('x (inertial)');
ylabel('y (inertial)');
title('plot of trajectory');
legend([walls_plot, vertex_plot, edge_plot, start_plot, goal_plot, traj_Plot], ...
    {'walls', 'vertices', 'edges', 'start', 'goal', 'trajectory'});
axis equal;

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = defaultRuntime;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
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
                   'beacon', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

maxV = 0.1;
wheel2Center = 0.13;


SetFwdVelAngVelCreate(Robot, 0,0);
tic


while toc < maxTime && gotopt <= length(waypoints)
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
        continue;
    end
    
    pose = dataStore.truthPose(end, 2:4);
    [fwdVel, angVel, gotopt] = visitWaypoints(waypoints, gotopt, closeEnough, pose, epsilon);
    [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center);
    SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    
    [px, py, ~] = OverheadLocalizationCreate(Robot);
    set(traj_Plot, 'XData', [get(traj_Plot, 'XData'), px], 'YData', [get(traj_Plot, 'YData'), py]);
    
    % pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0, 0);

end

function [mapBoundary] = calcMapBoundary(map)
xmin = min([map(1:4,1),map(1:4,3)],[],'all');
ymin = min([map(1:4,2),map(1:4,4)],[],'all');
xmax = max([map(1:4,1),map(1:4,3)],[],'all');
ymax = max([map(1:4,2),map(1:4,4)],[],'all');
mapBoundary = [xmin ymin xmax ymax];
end