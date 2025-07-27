function scan = simulateDepth(pose, map)
% simulateDepth - Simulate 1D lidar-style depth scan from a pose
%
% INPUT:
%   pose - [x, y, theta] of the particle
%   map  - Nx4 matrix of wall segments [x1 y1 x2 y2]
%
% OUTPUT:
%   scan - 1xN vector of distances in each direction

    scanAngles = linspace(27, -27, 10) * pi / 180;
    maxRange = 10.0;
    scan = zeros(1, length(scanAngles));

    for i = 1:length(scanAngles)
        beamAngle = wrapToPi(pose(3) + scanAngles(i));
        beamEnd = pose(1:2) + maxRange * [cos(beamAngle), sin(beamAngle)];

        minDist = maxRange;
        for j = 1:size(map, 1)
            wallStart = map(j, 1:2);
            wallEnd   = map(j, 3:4);
            [isect, x, y] = intersectPoint(pose(1), pose(2), beamEnd(1), beamEnd(2), ...
                                           wallStart(1), wallStart(2), wallEnd(1), wallEnd(2));
            if isect
                dist = norm([x, y] - pose(1:2));
                if dist < minDist
                    minDist = dist;
                end
            end
        end
        scan(i) = minDist;
    end
end
