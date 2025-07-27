function [mapBoundary] = calcMapBoundary(map)
    xmin = min([map(1:4,1),map(1:4,3)],[],'all');
    ymin = min([map(1:4,2),map(1:4,4)],[],'all');
    xmax = max([map(1:4,1),map(1:4,3)],[],'all');
    ymax = max([map(1:4,2),map(1:4,4)],[],'all');
    mapBoundary = [xmin ymin xmax ymax];
end