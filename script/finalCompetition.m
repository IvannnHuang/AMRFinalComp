function [dataStore] = finalCompetition(Robot,maxTime)

    addpath("maps\");
    addpath("plotting\");
    addpath("helper_functions\");
    
    %initialize variables
    % mapFile = 'map1_3credits.mat';
    mapFile = 'compMap.mat';
    map = load(mapFile).map;
    stayAwayPoints = load(mapFile).stayAwayPoints;
    optWalls = load(mapFile).optWalls;
    %sort points L->R
    waypoints = sortrows(load(mapFile).waypoints, 'ascend');
    ECwaypoints = sortrows(load(mapFile).ECwaypoints, 'ascend');

    %find locations of center points of optional walls and store in
    %wallCenters
    %locations of center points of optional walls
    wallCenters = ones(size(optWalls,1),2);
    for i = 1:size(optWalls,1)
        wallCenters(i,:) = [(optWalls(i,3)-optWalls(i,1))/2, (optWalls(i,4)-optWalls(i,2))/2];
        wallCenters(i,:) = wallCenters(i,:) + [min(optWalls(i,3),optWalls(i,1)), min(optWalls(i,4),optWalls(i,2))];
    end
    %sort points and walls R->L by center of the wall
    [wallCenters, idx] = sortrows(wallCenters, 'descend');
    optWalls = optWalls(idx,:);

    padding = 0.2;
    %add stay away points as obstacles to map
    for i = 1:size(stayAwayPoints,1)
        pt = stayAwayPoints(i,:);
        %add walls in a diamond shape around the stay away point a distance
        %padding from the point
        map = [map; pt + [padding,0], pt + [0,padding]; 
            pt + [0,padding], pt + [-padding, 0];
            pt + [-padding,0], pt + [0,-padding];
            pt + [0,-padding], pt + [padding, 0]];
    end

    % Robot setup
    defaultRuntime = 200;
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
    dataStore = struct('truthPose', [],'odometry', [], 'rsdepth', [], 'bump', [], 'beacon', [], 'traj', [], 'visitedWP', [], 'wallStates', []);

    % [px, py, pt] = OverheadLocalizationCreate(Robot);
    % truthPose = [px, py, pt];
    % init_pos = truthPose;
    
    [dataStore, init_pos] = initialPF(Robot,maxTime, mapFile, dataStore);
    truthPose = init_pos;
    closestDist = 1;
    for i = 1:size(waypoints,1)
        dist = norm([waypoints(i,1)-init_pos(1), waypoints(i,2)-init_pos(2)]);
        if dist < closestDist
            dataStore.visitedWP = [waypoints(i,1:2)];
        end
    end
    dataStore.traj = init_pos;
    goal_count = 1;
    if ismember(waypoints(goal_count, :), dataStore.visitedWP, 'rows')
        goal_count = goal_count + 1;
    end

    %stores walls states: -1=unknown, 0=not present, 1=present
    dataStore.wallStates = -1*ones(size(wallCenters,1), 1); 

    SetFwdVelAngVelCreate(Robot, 0,0);
    tic

    SetLEDsRoomba(Robot, 3, 0, 100); % LED green
    % go to regular waypoints
    while goal_count <= length(waypoints)
        goal = waypoints(goal_count, :);
        [dataStore, truthPose] = navigPF(Robot,maxTime, map, truthPose, goal, optWalls, dataStore, waypoints, ECwaypoints, 0.05);
        goal_count = goal_count+1;
        SetLEDsRoomba(Robot, 3, 100, 100); % LED green
        pause(0.5);
        SetLEDsRoomba(Robot, 3, 0, 100); % LED green
        % if waypoint has already been visited, skip it
        if goal_count < length(waypoints) && ismember(waypoints(goal_count, :), dataStore.visitedWP, 'rows')
            goal_count = goal_count + 1;
        end
    end

    % go to extra credit waypoints
    goal_count = 1;
    while goal_count <= length(ECwaypoints)
        % Move to the nearest waypoint for better path planning
        dists = sqrt(sum((waypoints - truthPose(1:2)).^2, 2));
        % Find index of the minimum distance
        [~, idx] = min(dists);
        goal = waypoints(idx, :);
        [dataStore, truthPose] = navigPF(Robot,maxTime, map, truthPose, goal, optWalls, dataStore, waypoints, ECwaypoints, 0.05);
        pause(0.5);
        goal = ECwaypoints(goal_count, :);
        [dataStore, truthPose] = navigPF(Robot,maxTime, map, truthPose, goal, optWalls, dataStore, waypoints, ECwaypoints, 0.05);
        goal_count = goal_count+1;
        SetLEDsRoomba(Robot, 3, 100, 100); % LED green
        pause(0.5);
        SetLEDsRoomba(Robot, 3, 0, 100); % LED green
        % if waypoint has already been visited, skip it
        if goal_count < length(ECwaypoints) && ismember(ECwaypoints(goal_count, :), dataStore.visitedWP, 'rows')
            goal_count = goal_count + 1;
        end
    end

    % go to optional walls
    goal_count = 1;
    while goal_count <= size(wallCenters, 1)
        % Move to the nearest waypoint for better path planning
        dists = sqrt(sum((waypoints - truthPose(1:2)).^2, 2));
        if (goal_count == 1)  % if robot at last wp
            % to not select itself
            dists(ismember(waypoints, dataStore.visitedWP(end, :), 'rows')) = Inf;
        end
        % Find index of the minimum distance
        [~, idx] = min(dists);
        goal = waypoints(idx, :);
        [dataStore, truthPose] = navigPF(Robot,maxTime, map, truthPose, goal, optWalls, dataStore, waypoints, ECwaypoints, 0.05);
        pause(0.5);
        goal = wallCenters(goal_count, :);
        [dataStore, truthPose, map, present] = goToWalls(Robot,maxTime, map, truthPose, goal, optWalls, wallCenters, dataStore);
        dataStore.wallStates(goal_count) = present;
        goal_count = goal_count+1;
    end

    %plot map with optional walls, visited waypoints
    figure(3);
    hold on;
    %plot map
    initMap = load(mapFile).map;
    for i = 1:size(initMap,1)
        plot([initMap(i,1),initMap(i,3)], [initMap(i,2),initMap(i,4)], 'k')
    end
    %plot optional walls
    for i = 1:size(optWalls,1)
        %undetermined walls drawn in red
        if dataStore.wallStates(i) == -1
            plot([optWalls(i,1),optWalls(i,3)], [optWalls(i,2), optWalls(i,4)], 'r');
        %walls determined to be present are drawn in black
        elseif dataStore.wallStates(i) == 1
            plot([optWalls(i,1),optWalls(i,3)], [optWalls(i,2), optWalls(i,4)], 'k');
        end
        %walls determined to not exist are not drawn
    end
    %plot visited waypoints
    for i = 1:size(dataStore.visitedWP,1)
        scatter(dataStore.visitedWP(:, 1), dataStore.visitedWP(:, 2), 'bx', 'LineWidth', 2);
    end
    %plot robot's trajectory
    plot(dataStore.traj(:,1), dataStore.traj(:,2), 'b');
end

