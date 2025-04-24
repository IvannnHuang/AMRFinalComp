function [head, edges] = buildForwardStar(V, E)
n = size(V, 1);
m = size(E, 1);
head = -1 * ones(n, 1);

% to: destination,  next: next edge index,  w: weight
edges(2*m) = struct('to', 0, 'next', -1, 'w', 0);

edgeIdx = 1;

for i = 1:m
    u = E(i, 1);
    v = E(i, 2);

    w = norm(V(u,:) - V(v,:)); % Euclidean distance

    % add u -> v
    edges(edgeIdx).to = v;
    edges(edgeIdx).w = w;
    edges(edgeIdx).next = head(u);
    head(u) = edgeIdx;
    edgeIdx = edgeIdx + 1;

    % add v -> u (if G is an undirected graph)
    edges(edgeIdx).to = u;
    edges(edgeIdx).w = w;
    edges(edgeIdx).next = head(v);
    head(v) = edgeIdx;
    edgeIdx = edgeIdx + 1;
end
end