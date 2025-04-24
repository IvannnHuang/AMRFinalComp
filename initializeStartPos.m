function [mu_t, sigma_t, theta_est] = initializeStartPos(mu_0, sigma_0, measurements, map)
% initializeStartPos: estimate robot position AND orientation using range
% measurements in unknown global directions.
%
% INPUTS:
%   mu_0           Initial 2D position estimate [x; y; theta]
%   sigma_0        Initial 2x2 covariance
%   measurements   NxM matrix of depth readings in M directions around robot
%   map            Wall map (N x 4): [x1, y1, x2, y2]
%
% OUTPUTS:
%   mu_t           Estimated position [x; y]
%   sigma_t        Estimated covariance (2x2)
%   theta_est      Estimated orientation (radians)

mu_t = mu_0(1:2);
sigma_t = sigma_0;

num_sensors = size(measurements, 2);
sensor_angles = linspace(0, 2*pi, num_sensors + 1);
sensor_angles(end) = [];

% Orientation hypotheses to test
num_orientations = 16;
orientations = linspace(0, 2*pi, num_orientations + 1);
orientations(end) = [];

Q_single = 0.1^2;  % measurement noise variance

theta_est_list = zeros(size(measurements,1), 1);  % record chosen orientation per step

for i = 1:size(measurements, 1)
    best_mu = mu_t;
    best_sigma = sigma_t;
    min_error = inf;
    best_theta = 0;

    for theta = orientations
        z_t = [];
        z_hat = [];
        H = [];

        for j = 1:num_sensors
            meas = measurements(i, j);
            if ~isnan(meas)
                global_angle = wrapTo2Pi(theta + sensor_angles(j));

                % Predict depth in global direction
                predicted_depth = depthPredict([mu_t; theta], map, [0, 0], global_angle);

                z_t(end+1,1) = meas;
                z_hat(end+1,1) = predicted_depth;

                % Measurement direction vector
                H(end+1,:) = [cos(global_angle), sin(global_angle)];
            end
        end

        if isempty(z_t)
            continue;
        end

        Q = eye(length(z_t)) * Q_single;

        K = sigma_t * H' / (H * sigma_t * H' + Q);
        mu_candidate = mu_t + K * (z_t - z_hat);
        sigma_candidate = (eye(2) - K * H) * sigma_t;

        error = norm(z_t - z_hat);

        if error < min_error
            min_error = error;
            best_mu = mu_candidate;
            best_sigma = sigma_candidate;
            best_theta = theta;
        end
    end

    mu_t = best_mu;
    sigma_t = best_sigma;
    theta_est_list(i) = best_theta;
end

% Final estimated orientation: either last, or average if you prefer smoothing
theta_est = wrapTo2Pi(theta_est_list(end));

end
