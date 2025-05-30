function particles = resetPF(pose)
    N = 500;  
    posStd = 0.0;   
    angleSpread = deg2rad(2);  
    particles = zeros(3, N);
    particles(1, :) = pose(1) + posStd * randn(1, N);  
    particles(2, :) = pose(2) + posStd * randn(1, N);  
    particles(3, :) = pose(3) + (2 * angleSpread * rand(1, N) - angleSpread);  
    particles(3, :) = mod(particles(3, :) + pi, 2*pi) - pi;
end 