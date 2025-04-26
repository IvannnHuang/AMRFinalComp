% PF: Performs one iteration of a particle filter.
%
%   INPUTS
%       particles         N-by-3 matrix of particle states [x, y, theta]
%       ut                Control input [d; phi] (distance traveled, angle turned)
%       depthMea          Vector of depth measurements
%       integrateOdom     Function handle for particle prediction
%       depthPredict      Function handle for weight update
%
%   OUTPUTS
%       new_particles     Updated N-by-3 matrix of particle states
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Ivan Huang

function [newParticles, p1] = PF(particles, odometry, depthMea, integrateOdom1, depthPredict, map, sensorOrigin, n_rs_rays, sigma)

addpath("maps\");
addpath("plotting\");
addpath("helper_functions\");

    numParticles = size(particles, 2);
    weights = zeros(1, numParticles);
    pred_off = 0.13*ones(1, n_rs_rays);
    
    for i = 1:numParticles
        % Step 1: Motion Update (Prediction)
        particles(:, i) = integrateOdom1(particles(:, i), odometry(1), odometry(2));

        % Step 2: Measurement Update
        pred_depth = depthPredict(particles(:, i), map, sensorOrigin, linspace(27, -27, n_rs_rays)*pi/180');
        residual = norm(pred_depth+2*pred_off - depthMea);
        error = sum(residual.^2);        
        weights(i) = exp(-error / (2 * sigma^2));
        if isnan(weights(i)) || isinf(weights(i))
            weights(i) = 1e-12;
        end
    end
    
    weights = weights / sum(weights);

    [topWeights, bestIndices] = maxk(weights, 5);
    p1 = particles(:, bestIndices(1));
    % if max(weights) < 1e-5
    %     newParticles = resetPF(p1);  % reset cloud around last estimate
    %     weights = ones(1, numParticles) / numParticles;
    % end

    % Step 3: Resampling
    newParticles = zeros(size(particles));
    cumulativeSum = cumsum(weights);
    step = 1 / numParticles;
    r = rand() * step;
    index = 1;
    for i = 1:numParticles
        u = r + (i - 1) * step;
        while u > cumulativeSum(index)
            index = index + 1;
        end
        newParticles(:, i) = particles(:, index);
    end
end

