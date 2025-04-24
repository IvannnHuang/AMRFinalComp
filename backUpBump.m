function [dataStore] = motionControl1(Robot, maxTime)
 % MOTIONCONTROL: Drives the robot while reacting to the bump sensor and 
 % switching between EKF (GPS or depth) and PF (depth) filtering.
 %
 %   dataStore = MOTIONCONTROL(Robot, maxTime, filterMethod)
 %
 %   INPUTS:
 %       Robot       Port configurations and robot name (from CreatePiInit)
 %       maxTime     Maximum time to run the program (in seconds)
 %
 %   OUTPUTS:
 %       dataStore   Struct containing logged data
 %
 %   Cornell University
 %   Autonomous Mobile Robots
 %   Lab 2
 %   Ivan Huang
 
 addpath("maps\");
 addpath("plotting\");
 addpath("helper_functions\");
 
 % filterMethod 1: EKF with GPS, 2: EKF with depth, 3: PF with depth
 filterMethod = 3;
 
 trace_selected = 0;
 
 if nargin < 2
     maxTime = 500;
 end
 
 % Global dataStore
 global dataStore;
 dataStore = struct('truthPose', [], 'odometry', [], 'rsdepth', [], ...
                    'bump', [], 'beacon', [], 'particles', [], 'pose', []);
 
 % Initialize robot state
 [noRobotCount, dataStore] = readStoreSensorData(Robot, 0, dataStore);
 if isempty(dataStore.truthPose)
     disp('Error: No initial pose data found.');
     return;
 end
 
 map = load('map1_3credits.mat').map;
 n_rs_rays = 10;
 sensor_pos = [0.13 0];
 
 % Initialize particle filter (PF) 
 dataStore.particles = resetPF([-3.5, 1.5, deg2rad(0)]);
 count = 0;
 
 map = load('map1_3credits.mat').map;
 mapBoundary = calcMapBoundary(map);
 robotRadius = 0.2;
 waypoints = load('map1_3credits.mat').waypoints;
 goal = waypoints(1, :);
 [px, py, ~] = OverheadLocalizationCreate(Robot);
 start = [px, py];
 
 fig1 = figure(1);
 hold on
 for i = 1:size(map, 1)
     if ~all(isnan(map(i,:))) && ~all(map(i,:) == 0)
         walls_plot = plot([map(i, 1), map(i, 3)], [map(i, 2), map(i, 4)], 'k', 'LineWidth', 1);
     end
 end
 
 axis equal;
 grid on;
 
 traj_Plot = plot(nan, nan, 'r-', 'LineWidth', 1.5);
 local_Plot = plot(nan, nan, 'b-', 'LineWidth', 1.5);
 
 xlabel('x (inertial)');
 ylabel('y (inertial)');
 title('plot of trajectory');
 axis equal;
 
 % Start robot movement
 SetFwdVelAngVelCreate(Robot, 0.2, 0); 
 
 tic;
 while toc < maxTime
     % Read sensor data
     [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
     delta = dataStore.odometry(end, 2:3)';  % distance moved
     depth = dataStore.rsdepth(end, 2:end)'; % depth reading
 
     if noRobotCount >= 3
         SetFwdVelAngVelCreate(Robot, 0, 0);
         disp('Lost localization, stopping robot.');
         break;
     end 
     
     % Handle bump sensor: Avoid obstacles
     if ~isempty(dataStore.bump) && any(dataStore.bump(end, 2:end))
         disp('Bump detected, changing direction.');
         SetFwdVelAngVelCreate(Robot, -0.2, 0); % Reverse briefly
         pause(0.5);
         SetFwdVelAngVelCreate(Robot, 0, -pi/6); % Rotate
         pause(0.5);
     else
         SetFwdVelAngVelCreate(Robot, 0.5, 0); % Continue moving
     end
 
     [dataStore.particles, pose] = PF(dataStore.particles, delta, depth, ...
                              @integrateOdom1, @depthPredict, map, sensor_pos, n_rs_rays);
     dataStore.pose = [dataStore.pose; pose'];
     % if (count == 30)
     %     dataStore.particles = resetPF(dataStore.pose(end, :));
     %     count = 0;
     % end
     % count = count + 1;
 
     set(traj_Plot, 'XData', [get(traj_Plot, 'XData'), dataStore.truthPose(end, 2)], 'YData', [get(traj_Plot, 'YData'), dataStore.truthPose(end, 3)]);
     set(local_Plot, 'XData', [get(local_Plot, 'XData'), dataStore.pose(end, 1)], 'YData', [get(local_Plot, 'YData'), dataStore.pose(end, 2)]);
 end
 
 % Stop the robot
 SetFwdVelAngVelCreate(Robot, 0, 0);
 end