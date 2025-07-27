function [dataStore, final_pose, map, present] = goToWalls(Robot,maxTime, map, start, goal, optWalls, wallCenters, dataStore)
% 
%   dataStore = goToWalls(Robot,maxTime, map, start, goal) sets the robot
%   on a path to go to the center of the next wall
% 
%   INPUTS
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data
%       final_pose  last position of robot

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Natalie Kaplan
    
    defaultRuntime = 1000;
    
    addpath("maps\");
    addpath("plotting\");
    addpath("helper_functions\");
    
    maxV = 0.1;
    wheel2Center = 0.13;
    noRobotCount = 0;

    present = 0;

    % Navigation setup
    mapBoundary = calcMapBoundary(map);
    robotRadius = 0.2;
    [V, E] = RRTwalls(map, mapBoundary, start(1:2), goal, robotRadius, 0.06);
    waypoints = findPath([], V, E, start, goal);
    gotopt = 1;
    epsilon = 0.2;
    closeEnough = 0.1;
    n_rs_rays = 10;
    sensor_pos = [0.13 0];
    
    % Initialize particle filter (PF) 
    particles = resetPF(start);
    
    fig1 = figure(1);
    cla;
    [vertex_plot, edge_plot] = plotRoadmap(V, E, fig1);
    hold on;
    for i = 1:size(map, 1)
        if ~all(isnan(map(i,:))) && ~all(map(i,:) == 0)
            walls_plot = plot([map(i, 1), map(i, 3)], [map(i, 2), map(i, 4)], 'k', 'LineWidth', 1);
        end
    end
    title('RRT');

    fig2 = figure(2);
    cla;
    hold on;
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
    wp_plot = scatter(dataStore.visitedWP(:, 1), dataStore.visitedWP(:, 2), 'bx', 'LineWidth', 2);
    
    xlabel('x (inertial)');
    ylabel('y (inertial)');
    title('plot of trajectory');
    legend([walls_plot, start_plot, goal_plot, traj_Plot, pf_Plot, wp_plot], ...
        {'walls', 'start', 'goal', 'trajectory', 'most weight', 'wp visited'});
    axis equal;
    
    SetFwdVelAngVelCreate(Robot, 0,0);
    tic
    
    %at this distance away from the wall, start tracking the bump
    %sensor
    closeToWall = 0.3;
    while toc < maxTime && gotopt <= length(waypoints)
        
        % READ & STORE SENSOR DATA
        [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
        delta = dataStore.odometry(end, 2:3)';  % distance moved
        depth = dataStore.rsdepth(end, 2:end)'; % depth reading
        bump = dataStore.bump(end, 2:end);
        
        % CONTROL FUNCTION (send robot commands)
        % if overhead localization loses the robot for too long, stop it
        if noRobotCount >= 3
            SetFwdVelAngVelCreate(Robot, 0,0);
            continue;
        end
        
        [particles, pose] = PF(particles, delta, bump, depth, ...
                              @integrateOdom1, @depthPredict, map, sensor_pos, n_rs_rays, 0.15, 0.05);
        [fwdVel, angVel, gotopt] = visitWaypoints(waypoints, gotopt, closeEnough, pose, epsilon);
        [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center);
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

        %if robot is close to the optional wall, track the bump sensor. 
        % If the bump sensor activates, the robot hit the optional wall
        %and therefore it is there. If the robot arrives at the waypoint (gets
        %closeEnough distance away) without activating the bump sensor, there
        %is no wall.
        if sqrt((pose(1)-goal(1))^2+(pose(2)-goal(2))^2) <= closeToWall && any(bump)
            [~,indices] = ismember(wallCenters, goal, 'rows');
            index = nonzeros(indices);
            map = [map; optWalls(index,:)];
            present = 1;
            break;
        end
        
        [px, py, ~] = OverheadLocalizationCreate(Robot);
        set(traj_Plot, 'XData', [get(traj_Plot, 'XData'), px], 'YData', [get(traj_Plot, 'YData'), py]);
        set(pf_Plot, 'XData', [get(pf_Plot, 'XData'), pose(1, end)], 'YData', [get(pf_Plot, 'YData'), pose(2, end)]);
        
        % pause(0.1);
    end
    
    final_pose = pose';

    % set forward and angular velocity to zero (stop robot) before exiting the function
    SetFwdVelAngVelCreate(Robot, 0, 0);

end
