function [dataStore] = navigPF(Robot,maxTime)

    addpath("maps\");
    addpath("plotting\");
    addpath("helper_functions\");

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
    dataStore = struct('truthPose', [],'odometry', [], 'rsdepth', [], 'bump', [], 'beacon', []);

    mapFile = 'map1_3credits.mat';
    map = load(mapFile).map;
    % waypoints = load(mapFile).waypoints;
    waypoints = [0, -3.5; -4.5, -3.5; -1.5, 3.5; -0.5, -0.5; 2.5, 1.5; 4.5 -1];
    goal_count = 1;
    goal = waypoints(goal_count, :);
    [px, py, pt] = OverheadLocalizationCreate(Robot);
    truthPose = [px, py, pt];
    SetFwdVelAngVelCreate(Robot, 0,0);
    tic
    
    while goal_count <= length(waypoints)
        [~, truthPose] = navigPF(Robot,maxTime, map, truthPose, goal);
        % [dataStore, truthPose] = initialPF(Robot,maxTime, mapFile);
        % if (truthPose(1)-goal(1) > 0.15 || truthPose(2)-goal(2) > 0.15)
        %     navigPF(Robot,maxTime, map, truthPose, goal)
        % else
        %     goal_count = goal_count+1;
        % end
        goal_count = goal_count+1;
        goal = waypoints(goal_count, :);
    end




