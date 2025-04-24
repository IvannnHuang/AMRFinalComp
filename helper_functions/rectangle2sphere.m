function sphere = rectangle2sphere(rect_pts)
    % rect_pts: 4x2 matrix, corners of rectangle
    % sphere: [x_center, y_center, radius]

    x_center = mean(rect_pts(:,1));
    y_center = mean(rect_pts(:,2));
    radius = sqrt((rect_pts(1,1)-x_center).^2 + (rect_pts(1,2)-y_center).^2);

    sphere = [x_center, y_center, radius];
end