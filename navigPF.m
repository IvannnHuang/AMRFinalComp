function[dataStore] = navigPF(Robot,maxTime)
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
    dataStore = struct('truthPose', [],'odometry', [], 'rsdepth', [], 'bump', [], 'beacon', []);
    noRobotCount = 0;
    
    maxV = 0.2;
    wheel2Center = 0.13;
    
    % Navigation setup
    % [dataStore, init_pose] = initialPF(Robot,maxTime);
    
    map = load('map1_3credits.mat').map;
    mapBoundary = calcMapBoundary(map);
    robotRadius = 0.2;
    waypoints = load('map1_3credits.mat').waypoints;
    goal = waypoints(1, :);
    % start = init_pose;
    start = [2.5,1.5, deg2rad(0)];
    [V, E] = RRTwalls(map, mapBoundary, start(1:2), goal, robotRadius);
    waypoints = findPath([], V, E, start, goal);
    gotopt = 1;
    epsilon = 0.2;
    closeEnough = 0.1;
    n_rs_rays = 10;
    sensor_pos = [0.13 0];
    
    % Initialize particle filter (PF) 
    particles = resetPF(start);
    sigma = 0.5;
    
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
    
    traj_Plot = plot(nan, nan, 'r-', 'LineWidth', 1.5);
    pf_Plot = plot(nan, nan, 'b-', 'LineWidth', 1.5);
    goal_plot = scatter(goal(1), goal(2), 'gx', 'LineWidth', 2);
    start_plot = scatter(start(1), start(2), 'rx', 'LineWidth', 2);
    
    xlabel('x (inertial)');
    ylabel('y (inertial)');
    title('plot of trajectory');
    legend([walls_plot, start_plot, goal_plot, traj_Plot, pf_Plot], ...
        {'walls', 'start', 'goal', 'trajectory', 'most weight'});
    axis equal;
    
    SetFwdVelAngVelCreate(Robot, 0,0);
    tic
    
    while toc < maxTime && gotopt <= length(waypoints)
        
        % READ & STORE SENSOR DATA
        [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
        delta = dataStore.odometry(end, 2:3)';  % distance moved
        depth = dataStore.rsdepth(end, 2:end)'; % depth reading
        
        % CONTROL FUNCTION (send robot commands)
        % if overhead localization loses the robot for too long, stop it
        if noRobotCount >= 3
            SetFwdVelAngVelCreate(Robot, 0,0);
            continue;
        end
        
        % truthPose = dataStore.truthPose(end, 2:4);

        [particles, pose] = PF(particles, delta, depth, ...
                              @integrateOdom1, @depthPredict, map, sensor_pos, n_rs_rays, sigma);
        [fwdVel, angVel, gotopt] = visitWaypoints(waypoints, gotopt, closeEnough, pose, epsilon);
        [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        
        [px, py, ~] = OverheadLocalizationCreate(Robot);
        set(traj_Plot, 'XData', [get(traj_Plot, 'XData'), px], 'YData', [get(traj_Plot, 'YData'), py]);
        set(pf_Plot, 'XData', [get(pf_Plot, 'XData'), pose(1, end)], 'YData', [get(pf_Plot, 'YData'), pose(2, end)]);
        
        % pause(0.1);
    end
    
    % set forward and angular velocity to zero (stop robot) before exiting the function
    SetFwdVelAngVelCreate(Robot, 0, 0);

end
