function [vertex_plot, edge_plot] = plotRoadmap(V, E, figHdl)
    figure(figHdl);
    hold on;
    axis equal;

    for i = 1:size(E,1)
        idx1 = E(i,1);
        idx2 = E(i,2);
        p1 = V(idx1, :);
        p2 = V(idx2, :);
        edge_plot = plot([p1(1), p2(1)], [p1(2), p2(2)], 'b-', 'LineWidth', 1.5);
    end

    vertex_plot = plot(V(:,1), V(:,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 3);

    hold off;
end
