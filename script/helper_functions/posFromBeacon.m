function pos = posFromBeacon(data, beaconLoc)
% posFromBeacon: estimates the position of the robot using beacon measurements      
%   INPUTS
%       data                    data from RealSenseTag - array [dt id x y rot]
%       beaconLoc               locations of each beacon on map - array [id x y]
%
%   OUTPUTS
%       pos         The estimated pose of the robot [x y theta]

if isempty(data)
    error('No beacon data provided.');
end

tagID = data(1,2);
relCam = data(1,3:4);  % [x_cam, y_cam]
beaconTheta = data(1,5);

% Camera frame to robot frame (flip y and add offset)
relRobot = [relCam(2); relCam(1)];
cameraOffset = [0.08; 0];     % 8cm in front of robot center
relRobot = relRobot + cameraOffset;

% Get beacon global position
idx = find(beaconLoc(:,1) == tagID, 1);
if isempty(idx)
    error('Beacon ID %d not found in beaconLoc.', tagID);
end
beaconWorld = beaconLoc(idx, 2:3)';

% The beacon is at beaconWorld and is facing `beaconTheta`
% That means the *measured* relRobot vector is pointing FROM robot TO beacon
% We want to rotate that relative vector by beacon's orientation to get the *absolute direction* from robot to beacon
R_beacon = [cos(beaconTheta), -sin(beaconTheta); sin(beaconTheta), cos(beaconTheta)];
relWorld = R_beacon * relRobot;

% Robot position = beacon position - transformed rel vector
robotPos = beaconWorld - relWorld;

% Orientation estimation: robot is facing *toward* the beacon (opposite of relRobot in robot frame)
% So, compute expected direction robot is facing:
theta = wrapToPi(atan2(relWorld(2), relWorld(1)) - atan2(relRobot(2), relRobot(1)));

% Output pose
pos = [robotPos' theta];

end