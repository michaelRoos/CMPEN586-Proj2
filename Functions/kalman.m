function [output] = kalman(dataset, model)
%KALMAN Summary of this function goes here
%   Detailed explanation goes here
[frameLength, ~] = size(dataset);
output = dataset;
% Experimental threshold to detect outlier missclassified points
G_threshold = 10;
% Initial point to start the kalman filter
x0 = dataset(1,1);
y0 = dataset(1,2);

% Set Kalman parameters based on chosen motion model
if strcmp(model,'Constant_Velocity')
    % Constant Velocity update
    F = [   
            1 0 1 0;    ...
            0 1 0 1;    ...
            0 0 1 0;    ...
            0 0 0 1
        ];
    % Select postion components
    H = [   
            1 0 0 0;    ...
            0 1 0 0
        ];
    % Proper size identity matrix
    I = eye(4);
    
    % Uncertainty in the motion model prediction
    Q = diag([.1 .1 1 1]);
    % Uncertainty in the measurement prediction
    R = diag([10 10]);
    
    % Initial state as stationary first observed point
    x = [x0 y0 0 0]';
    % Initial distribution as independent multivariate with std 1
    P = diag([1 1 1 1]);
else
    % Constant Acceleration update
    F = [   
            1 0 1 0 0.5 0;  ...
            0 1 0 1 0 0.5;  ...
            0 0 1 0 1 0;    ...
            0 0 0 1 0 1;    ...
            0 0 0 0 1 0;    ...
            0 0 0 0 0 1;
        ];
    % Select postion components
    H = [
            1 0 0 0 0 0;    ...
            0 1 0 0 0 0
        ];
    % Proper size identity matrix
    I = eye(6);
    
    % Uncertainty in the motion model prediction
    Q = diag([1 1 1 1 1 1]);
    % Uncertainty in the measurement prediction
    R = diag([10 10]);
    
    % Initial state as stationary first observed point
    x = [x0 y0 0 0 0 0]';
    % Initial distribution as independent multivariate with std 1
    P = diag([1 1 1 1 1 1]);
   
end

for frameIndex = (1:frameLength)
    % Predicted next state and distribution from motion model
    x_ = F * x;
    P_ = F * P * F' + Q;
     
    
    % Next observed point
    observation = squeeze(dataset(frameIndex,:));
    % Delta between observerd and predicted point
    y = [observation(1) observation(2)]' - H * x_;
    % Weight observation noise by inverse exponential of confidence
    R_ = R * exp(-observation(3));
    % Distribution w.r.t just position
    S = H * P_ * H' + R_;
    % Calculate elliptical gate distance between predected and observed
    % G is the exponent in distribution. Probability = exp(-1/2 * G)
    G = y' / S * y;
    % If the gate is too large, the point must be a misdetect
    % Ignore the observed point and retain predicted, but expand uncertainy
    if G > G_threshold
        x = x_;
        P = P_ + Q;
    % If the gate is small enough, update the posterior with observation
    else 
        % Modified Kalman Gain
        K = P_ * H' / (S);
        % Update state estimate
        x = x_ + K*y;
        % Update distribution estimate
        P = (I - K * H) * P_;
    end
    % Save MAP estimator
    output(frameIndex,1:2) = H * x;
end
end

