function angleWrapped = wrapToPi(angle)
% wrapToPi wraps an angle in radians to [-pi, pi]
    angleWrapped = mod(angle + pi, 2*pi) - pi;
end