function [dataStore, init_pose] = initialPF(Robot,maxTime, mapFile)
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

% mapFile = 'map1_3credits.mat';
map = load(mapFile).map;
waypoints = load(mapFile).waypoints;
robotRadius = 0.2;
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

traj_Plot = quiver(nan, nan, nan, nan, 'MaxHeadSize',2,'Color','r','LineWidth',1.5);
pf_Plot = quiver(nan, nan, nan, nan, 'MaxHeadSize',2,'Color','b','LineWidth',1.5);
particle_Plot = scatter(nan, nan, 50, 'r.');  % tiny red dots

xlabel('x (inertial)');
ylabel('y (inertial)');
title('plot of trajectory');
legend([walls_plot, traj_Plot, pf_Plot, particle_Plot], ...
    {'walls', 'trajectory', 'most weight', 'particles'});
axis equal;

% Robot setup
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = defaultRuntime;
end
try 
    CreatePort=Robot.CreatePort;
catch
    CreatePort = Robot;
end
global dataStore;
dataStore = struct('truthPose', [],'odometry', [],'rsdepth', [],'bump', [],'beacon', []);
noRobotCount = 0;
maxV = 0.2;
wheel2Center = 0.13;
SetFwdVelAngVelCreate(Robot, 0,0);
tic

particles = startPF(waypoints);

count = 0;
while toc < maxTime && count < 19
    pause(0.1);
    % turn 360 degree
    if (count < 19)
        turnAngle(Robot, robotRadius, 11.65);
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

    [particles, pose] = PF1(particles, delta, depth, ...
                          @integrateOdom1, @depthPredict, map, sensor_pos, n_rs_rays, 2);

    [px, py, pt] = OverheadLocalizationCreate(Robot);
    set(traj_Plot, 'XData', px, ...
                  'YData', py, ...
                  'UData', 0.3*cos(pt), ...
                  'VData', 0.3*sin(pt));
    set(pf_Plot, 'XData', pose(1), ...
                  'YData', pose(2), ...
                  'UData', 0.3*cos(pose(3)), ...
                  'VData', 0.3*sin(pose(3)));
    set(particle_Plot, 'XData', particles(1,:), ...
                   'YData', particles(2,:));

end
init_pose = pose';

SetFwdVelAngVelCreate(Robot, 0, 0);
end