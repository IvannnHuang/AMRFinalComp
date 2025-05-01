function [dataStore, final_pose] = navigPF(Robot,maxTime, map, start, goal, optWalls, dataStore, wps, ECwaypoints, stepSize)
    
    defaultRuntime = 1000;
    
    addpath("maps\");
    addpath("plotting\");
    addpath("helper_functions\");
    
    maxV = 0.1;
    wheel2Center = 0.13;
    noRobotCount = 0;

    % Navigation setup
    mapBoundary = calcMapBoundary(map);
    robotRadius = 0.2;
    [V, E] = RRTwalls(map, mapBoundary, start(1:2), goal, robotRadius, stepSize);
    waypoints = findPath([], V, E, start, goal);
    gotopt = 1;
    epsilon = 0.2;
    closeEnough = 0.1;
    n_rs_rays = 10;
    sensor_pos = [0.13 0];
    closestDist = 0.1;
    
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
    hold on;
    for i = 1:size(map, 1)
        if ~all(isnan(map(i,:))) && ~all(map(i,:) == 0)
            walls_plot = plot([map(i, 1), map(i, 3)], [map(i, 2), map(i, 4)], 'k', 'LineWidth', 1);
        end
    end
    
    axis equal;
    grid on;
    
    traj_Plot = plot(nan, nan, 'r-', 'LineWidth', 1.5);
    pf_Plot = plot(nan, nan, 'b-', 'LineWidth', 1);
    goal_plot = scatter(goal(1), goal(2), 'gx', 'LineWidth', 2);
    start_plot = scatter(start(1), start(2), 'rx', 'LineWidth', 2);
    wp_plot = scatter(dataStore.visitedWP(:, 1), dataStore.visitedWP(:, 2), 'bx', 'LineWidth', 2);
    for i = 1:size(optWalls,1)
        if dataStore.wallStates(i) == -1
            plot([optWalls(i,1),optWalls(i,3)], [optWalls(i,2), optWalls(i,4)], 'r');
        %walls determined to be present are drawn in black
        % elseif dataStore.wallStates(i) == 1
        %     plot([optWalls(i,1),optWalls(i,3)], [optWalls(i,2), optWalls(i,4)], 'k');
        end
        %walls determined to not exist are not drawn
    end
    
    xlabel('x (inertial)');
    ylabel('y (inertial)');
    title('plot of trajectory');
    legend([walls_plot, start_plot, goal_plot, traj_Plot, pf_Plot, wp_plot], ...
        {'walls', 'start', 'goal', 'trajectory', 'most weight', 'wp visited'});
    axis equal;
    
    SetFwdVelAngVelCreate(Robot, 0,0);
    tic
    
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
        
        % truthPose = dataStore.truthPose(end, 2:4);

        [particles, pose] = PF(particles, delta, bump, depth, ...
                              @integrateOdom1, @depthPredict, map, sensor_pos, n_rs_rays, 0.15, 0.03);
        [fwdVel, angVel, gotopt] = visitWaypoints(waypoints, gotopt, closeEnough, pose, epsilon);
        [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center);
        dataStore.traj = [dataStore.traj;  pose'];
        % if pass by other waypoints
        for i = 1:size(wps,1)
            dist = norm([wps(i,1)-pose(1, end), wps(i,2)-pose(2, end)]);
            if dist < closestDist && ~ismember(wps(i,:), dataStore.visitedWP, 'rows')
                dataStore.visitedWP = [dataStore.visitedWP; wps(i,:)];
                SetLEDsRoomba(Robot, 3, 100, 100);
                pause(0.5);
                SetLEDsRoomba(Robot, 3, 0, 100);
                pause(0.5);
            end
        end
        for i = 1:size(ECwaypoints,1)
            dist = norm([ECwaypoints(i,1)-pose(1, end), ECwaypoints(i,2)-pose(2, end)]);
            if dist < closestDist && ~ismember(ECwaypoints(i,:), dataStore.visitedWP, 'rows')
                dataStore.visitedWP = [dataStore.visitedWP; ECwaypoints(i,:)];
                SetLEDsRoomba(Robot, 3, 100, 100);
                pause(0.5);
                SetLEDsRoomba(Robot, 3, 0, 100);
                pause(0.5);
            end
        end
        % if robot get stucked
        pose = pose';
        if (length(dataStore.traj)>=100) 
            if (dataStore.traj(length(dataStore.traj)-10, 1:2) == pose(1:2))
                turnAngle(Robot, robotRadius, 30);
            end
        end
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        
        [px, py, ~] = OverheadLocalizationCreate(Robot);
        set(traj_Plot, 'XData', [get(traj_Plot, 'XData'), px], 'YData', [get(traj_Plot, 'YData'), py]);
        set(pf_Plot, 'XData', [get(pf_Plot, 'XData'), pose(1)], 'YData', [get(pf_Plot, 'YData'), pose(2)]);
        
        % pause(0.1);
    end
    dataStore.visitedWP = [dataStore.visitedWP; goal];
    dataStore.visitedWP = unique(dataStore.visitedWP, 'rows');
    
    final_pose = pose;
   
    % set forward and angular velocity to zero (stop robot) before exiting the function
    SetFwdVelAngVelCreate(Robot, 0, 0);

end
