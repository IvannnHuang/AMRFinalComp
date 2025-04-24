function sphere_world = rectangle_walls_to_sphere_world(walls)
rectangles = {};
visited = false(size(walls, 1), 1);

% Extract rectangles from walls
for i = 1:size(walls,1)
    if visited(i)
        continue;
    end
    rect_pts = [walls(i, 1:2); walls(i, 3:4)];
    visited(i) = true;
    
    for j = i+1:size(walls,1)
        if visited(j)
            continue;
        end
        w_pts = [walls(j, 1:2); walls(j, 3:4)];
        if any(ismember(w_pts, rect_pts, 'rows'))
            rect_pts = unique([rect_pts; w_pts], 'rows', 'stable');
            visited(j) = true;
        end
    end

    rectangles{end+1} = rect_pts;
end

k = numel(rectangles);
sphere_world = zeros(k,3);
for idx = 1:k
    sphere_world(idx,:) = rectangle2sphere(rectangles{idx});
end
end