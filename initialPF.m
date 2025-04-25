function[dataStore] = initialPF(Robot,maxTime)
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
%   Huang, Ivan

defaultRuntime = 1000;

addpath("maps\");
addpath("plotting\");
addpath("helper_functions\");

mapFile = 'map1_3credits.mat';
map = load(mapFile).map;
mapBoundary = calcMapBoundary(map);
robotRadius = 0.2;
waypoints = load(mapFile).waypoints;
goal = waypoints(1, :);
[px, py, ~] = OverheadLocalizationCreate(Robot);
start = [px, py];
[V, E] = RRTwalls(map, mapBoundary, start, goal, robotRadius);
gotopt = 1;
epsilon = 0.2;
closeEnough = 0.1;
n_rs_rays = 10;
sensor_pos = [0.13 0];

fig1 = figure(1);
% [vertex_plot, edge_plot] = plotRoadmap(V, E, fig1);
hold on
for i = 1:size(map, 1)
    if ~all(isnan(map(i,:))) && ~all(map(i,:) == 0)
        walls_plot = plot([map(i, 1), map(i, 3)], [map(i, 2), map(i, 4)], 'k', 'LineWidth', 1);
    end
end

axis equal;
grid on;

% traj_Plot = plot(nan, nan, 'r-', 'LineWidth', 1.5);
% pf_Plot = plot(nan, nan, 'b-', 'LineWidth', 1.5);
traj_Plot = quiver(nan, nan, nan, nan, 'MaxHeadSize',2,'Color','r','LineWidth',1.5);
pf_Plot = quiver(nan, nan, nan, nan, 'MaxHeadSize',2,'Color','b','LineWidth',1.5);
particle_Plot = scatter(nan, nan, 50, 'r.');  % tiny red dots

xlabel('x (inertial)');
ylabel('y (inertial)');
title('plot of trajectory');
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
                   'beacon', [], ...
                   'particles', [], ...
                   'pose', []);

% Initialize particle filter (PF) 
dataStore.particles = startPF(waypoints);

% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

maxV = 0.2;
wheel2Center = 0.13;


SetFwdVelAngVelCreate(Robot, 0,0);
tic


count = 0;
while toc < maxTime 
    pause(0.1);
    % turn 360 degree
    if (count < 20)
        turnAngle(Robot, robotRadius, 12);
        count = count + 1;
    end

    % READ & STORE SENSOR DATA
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
    delta = dataStore.odometry(end, 2:3)';  % distance moved
    depth = dataStore.rsdepth(end, 2:end)'; % depth reading

    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
        continue;
    end

    [dataStore.particles, pose] = PF1(dataStore.particles, delta, depth, ...
                          @integrateOdom1, @depthPredict, map, sensor_pos, n_rs_rays);
    dataStore.pose = [dataStore.pose; pose'];

    [px, py, pt] = OverheadLocalizationCreate(Robot);
    set(traj_Plot, 'XData', px, ...
                  'YData', py, ...
                  'UData', 0.3*cos(pt), ...
                  'VData', 0.3*sin(pt));
    set(pf_Plot, 'XData', pose(1), ...
                  'YData', pose(2), ...
                  'UData', 0.3*cos(pose(3)), ...
                  'VData', 0.3*sin(pose(3)));
    set(particle_Plot, 'XData', dataStore.particles(1,:), ...
                   'YData', dataStore.particles(2,:));
end
SetFwdVelAngVelCreate(Robot, 0, 0);
end