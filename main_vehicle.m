%% UKF Part
% True Initial state: xhat[k|k-1]
initialState = [6500.4; 349.14; -1.8093; -6.7967];
% Guess of intiial state (Not same in general)
initialStateGuess = [6500.4; 349.14; -1.8093; -6.7967]; % xhat[k|k-1]
% Guess of initial covariance
initialCovarianceGuess = diag([10e-6 10e-6 10e-6 10e-6]); 
% Construct the filter
ukf = unscentedKalmanFilter(...
    @vehicleStateFcn,... % State transition function
    @vehicleMeasurementFcn,... % Measurement function
    initialStateGuess);

% Covariance of the process noise 
ukf.ProcessNoise = diag([0 0 2.4064e-5 2.4064e-5]);

% Initial covariance pi_o
ukf.StateCovariance = initialCovarianceGuess;

% Covariance Matrix of the measurement noise v[k]
R = diag([1e-3 17e-3]);
ukf.MeasurementNoise = R;

% Get true trajectory for x[1:4] noise-less measurements
% ODE update rate is every 100ms
T = 0.10; % [s] Filter sample time
% Siumlate for a time of 200s
timeVector = 0:T:200;
% Get true noiseless samples
[~, xTrue]=ode45(@vehicleStateFcnContinuous2,timeVector,initialState);
%Plot them
%plot(xTrue(:, 3), xTrue(:, 4))
%ylim([-200 500])
%xlim([6350 6500])

% Corrupt clean samples using measurement noise covariance (This is known to designer)
rng(1); % Fix the random number generator for reproducible results
yTrue = vehicleMeasurementFcn2(xTrue);
% sqrt(R): Standard deviation of noise
yMeas = yTrue + randn(size(yTrue))*sqrt(R);

Nsteps = size(yMeas, 1); % Number of time steps
xCorrectedUKF = zeros(Nsteps, 4); % Corrected state estimates
PCorrected = zeros(Nsteps, 4, 4); % Corrected state estimation error covariances
e = zeros(Nsteps, 2); % Residuals (or innovations)

for k=1:Nsteps
    % Let k denote the current time.
    %
    % Residuals (or innovations): Measured output - Predicted output
    e(k, :) = yMeas(k, :) - vehicleMeasurementFcn(ukf.State); % ukf.State is x[k|k-1] at this point
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    [xCorrectedUKF(k,:), PCorrected(k,:,:)] = correct(ukf,yMeas(k, :));
    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    predict(ukf);
end

%% EKF part 