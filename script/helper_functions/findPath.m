function path = findPath(obstacleVerts, V, E, start, goal, add_start_goal)
if nargin < 6
    add_start_goal = 0;
end
if add_start_goal
    [V, E, startIdx] = addPointToGraph(obstacleVerts, V, E, start);
    [V, E, goalIdx] = addPointToGraph(obstacleVerts, V, E, goal);
else
    startIdx = 1;
    goalIdx = size(V, 1);
end

[head, edges] = buildForwardStar(V, E);

% Dijkstra
n = size(V, 1);
dist = inf(1, n);
prev = -1 * ones(1, n);
visited = false(1, n);

dist(startIdx) = 0;

for i = 1:n
    u = -1;
    minDist = inf;
    for j = 1:n
        if ~visited(j) && dist(j) < minDist
            u = j;
            minDist = dist(j);
        end
    end
    if u == -1
        break;
    end

    visited(u) = true;

    if u == goalIdx
        break
    end

    idx = head(u);
    while idx ~= -1
        v = edges(idx).to;
        w = edges(idx).w;
        if dist(v) > dist(u) + w
            dist(v) = dist(u) + w;
            prev(v) = u;
        end
        idx = edges(idx).next;
    end
end

path = [];
idx = goalIdx;
while idx ~= -1
    path = [path; V(idx, :)];
    idx = prev(idx);
end
path = flip(path);

end

function [V, E, ptIdx] = addPointToGraph(obstacleVerts, V, E, pt)
    minDist = inf;
    bestIdx = -1;

    for i = 1:size(V, 1)
        pi = V(i, :);

        if ~checkCollision(pi, pt, obstacleVerts)
            d = norm(pi - pt);
            if d < minDist
                minDist = d;
                bestIdx = i;
            end
        end
    end

    if bestIdx == -1
        warning('No valid edge found: the point is isolated.');
        return;
    end

    V = [V; pt];
    ptIdx = size(V, 1);
    E = [E; bestIdx, ptIdx];
end

function collision = checkCollision(p1, p2, obstacleVerts)
    collision = false;
    for i = 1:length(obstacleVerts)
        poly = obstacleVerts{i};
        for j = 1:size(poly, 1)
            a = poly(j, :);
            b = poly(mod(j, size(poly, 1)) + 1, :);
            [isect, ~, ~] = intersectPoint(a(1), a(2), b(1), b(2), p1(1), p1(2), p2(1), p2(2));
            if isect
                collision = true;
                return;
            end
        end
    end
end