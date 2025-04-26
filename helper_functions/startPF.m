function particles = startPF(waypoints)
    N = 4200;  % total particles
    k = length(waypoints);  % number of candidates
    particles = zeros(3, N);
    particlesPerWP = floor(N / k);
    
    for i = 1:k
        idx = (i-1)*particlesPerWP + 1 : i*particlesPerWP;
        particles(1, idx) = waypoints(i, 1) + 0.02 * randn(1, particlesPerWP);  % x
        particles(2, idx) = waypoints(i, 2) + 0.02 * randn(1, particlesPerWP);  % y
        % particles(3, idx) = 0;  % theta
        particles(3, idx) = deg2rad(360) * randn(1, particlesPerWP);  % theta
    end
end 