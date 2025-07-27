function [V, E] = RRTwalls(walls, mapBoundary, start, goal, robotRadius, stepSize)
maxNodes = 1000;
xmin = mapBoundary(1) + robotRadius;
ymin = mapBoundary(2) + robotRadius;
xmax = mapBoundary(3) - robotRadius;
ymax = mapBoundary(4) - robotRadius;
stepSize = max(xmax - xmin, ymax - ymin) * stepSize;
goalThreshold = stepSize;

V = start;
E = [];

for i = 1:maxNodes
    sample = [rand * (xmax - xmin) + xmin, rand * (ymax - ymin) + ymin];

    if ~isPointValid(sample, walls, robotRadius)
        continue;
    end

    dists = vecnorm(V - sample, 2, 2);
    [~, nearestIdx] = min(dists);
    nearest = V(nearestIdx, :);

    dir = sample - nearest;
    dist = norm(dir);
    dir = dir / dist * stepSize;
    newPt = nearest + dir;

    if newPt(1) < xmin || newPt(1) > xmax || newPt(2) < ymin || newPt(2) > ymax
        continue;
    end

    if isPointValid(newPt, walls, robotRadius) && ...
        isEdgeValid(nearest, newPt, walls, robotRadius)
        V = [V; newPt];
        E = [E; nearestIdx, size(V, 1)];
        if norm(newPt - goal) < goalThreshold && ...
           isEdgeValid(newPt, goal, walls, robotRadius)
            V = [V; goal];
            E = [E; size(V, 1)-1, size(V, 1)];
            break;
        end
    end
end
end

function valid = isPointValid(pt, walls, robotRadius)
    valid = true;
    for i = 1:size(walls, 1)
        if pointToSegmentDistance(pt, walls(i, 1:2), walls(i, 3:4)) < robotRadius
            valid = false;
            return
        end
    end
end

function valid = isEdgeValid(p1, p2, walls, robotRadius)
    v = p2 - p1;
    L = norm(v);
    if L == 0
        valid = false;
        return;
    end
    v_unit = v / L;
    u = [-v_unit(2), v_unit(1)];
    A = p1 + robotRadius * u;
    B = p1 - robotRadius * u;
    C = p2 - robotRadius * u;
    D = p2 + robotRadius * u;
    
    valid = true;
    for j = 1:size(walls, 1)
        if isIntersect(walls(j, 1:2), walls(j, 3:4), A, D) || ...
            isIntersect(walls(j, 1:2), walls(j, 3:4), B, C)
            valid = false;
            return;
        end
    end
end

function d = pointToSegmentDistance(pt, a, b)
    v = b - a;
    w = pt - a;
    c1 = dot(w,v);
    if c1 <= 0
        d = norm(pt - a);
        return;
    end
    c2 = dot(v,v);
    if c2 <= c1
        d = norm(pt - b);
        return;
    end
    t = c1 / c2;
    proj = a + t*v;
    d = norm(pt - proj);
end

function isect = isIntersect(p1, p2, p3, p4)
    [isect, ~, ~] = intersectPoint(p1(1), p1(2), p2(1), p2(2), p3(1), p3(2), p4(1), p4(2));
end