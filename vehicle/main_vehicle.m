%% UKF Part
% Your initial state guess at time k, utilizing measurements up to time k-1: xhat[k|k-1]
% True Initial state: xhat[k|k-1]
initialState = [6500.4; 349.14; -1.8093; -6.7967; 0.6932];
% Guess of intiial state (Not same in general)
initialStateGuess = [6500.4; 349.14; -1.8093; -6.7967; 0]; % xhat[k|k-1]
% Guess of initial covariance
initialCovarianceGuess = diag([1e-6 1e-6 1e-6 1e-6 1]); 
% Construct the filter
ukf = unscentedKalmanFilter(...
    @vehicleStateFcn,... % State transition function
    @vehicleMeasurementFcn,... % Measurement function
    initialStateGuess);

% Covariance of the process noise 
ukf.ProcessNoise = diag([0 0 2.4064e-5 2.4064e-5 0]);

% Initial covariance pi_o
ukf.StateCovariance = initialCovarianceGuess;

% Covariance Matrix of the measurement noise v[k]
R = diag([1e-3 17e-3]);
ukf.MeasurementNoise = R;

% Get true trajectory for x[1:4] noise-less measurements
% ODE update rate is every 100ms
T = 0.05; % [s] Filter sample time
% Siumlate for a time of 200s
timeVector = 0:T:200;
% Get true noiseless samples
[~, xTrue]=ode45(@vehicleStateFcnContinuous2,timeVector,initialState);

% Corrupt clean samples using measurement noise covariance (This is known to designer)
rng(1); % Fix the random number generator for reproducible results
yTrue = vehicleMeasurementFcn2(xTrue);
% sqrt(R): Standard deviation of noise
yMeas = yTrue + randn(size(yTrue))*sqrt(R);

Nsteps = size(yMeas, 1); % Number of time steps
xCorrectedUKF = zeros(Nsteps, 5); % Corrected state estimates
PCorrectedUKF = zeros(Nsteps, 5, 5); % Corrected state estimation error covariances
eUKF = zeros(Nsteps, 2); % Residuals (or innovations)

for k=1:Nsteps
    % Let k denote the current time.
    %
    % Residuals (or innovations): Measured output - Predicted output
    eUKF(k, :) = yMeas(k, :) - vehicleMeasurementFcn(ukf.State); % ukf.State is x[k|k-1] at this point
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    [xCorrectedUKF(k,:), PCorrectedUKF(k,:,:)] = correct(ukf,yMeas(k, :));
    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    predict(ukf);
end

%% EKF Part
%Your initial state guess at time k, utilizing measurements up to time k-1: xhat[k|k-1]
% True Initial state: xhat[k|k-1]
initialState = [6500.4; 349.14; -1.8093; -6.7967; 0.6932];
% Guess of intiial state (Not same in general)
initialStateGuess = [6500.4; 349.14; -1.8093; -6.7967; 0]; % xhat[k|k-1]
% Guess of initial covariance
initialCovarianceGuess = diag([1e-6 1e-6 1e-6 1e-6 1]); 
% Construct the filter
ekf = extendedKalmanFilter(...
    @vehicleStateFcn,... % State transition function
    @vehicleMeasurementFcn,... % Measurement function
    initialStateGuess);

% Covariance of the process noise 
ukf.ProcessNoise = diag([0 0 2.4064e-5 2.4064e-5 0]);

% Initial covariance pi_o
ukf.StateCovariance = initialCovarianceGuess;

% Covariance Matrix of the measurement noise v[k]
R = diag([1e-3 17e-3]);
ukf.MeasurementNoise = R;

% Get true trajectory for x[1:4] noise-less measurements
% ODE update rate is every 100ms
T = 0.05; % [s] Filter sample time
% Siumlate for a time of 200s
timeVector = 0:T:200;
% Get true noiseless samples
[~, xTrue]=ode45(@vehicleStateFcnContinuous2,timeVector,initialState);

% Corrupt clean samples using measurement noise covariance (This is known to designer)
rng(1); % Fix the random number generator for reproducible results
yTrue = vehicleMeasurementFcn2(xTrue);
% sqrt(R): Standard deviation of noise
yMeas = yTrue + randn(size(yTrue))*sqrt(R);

Nsteps = size(yMeas, 1); % Number of time steps
xCorrectedEKF = zeros(Nsteps, 5); % Corrected state estimates
PCorrectedEKF = zeros(Nsteps, 5, 5); % Corrected state estimation error covariances
eEKF = zeros(Nsteps, 2); % Residuals (or innovations)

for k=1:Nsteps
    % Let k denote the current time.
    %
    % Residuals (or innovations): Measured output - Predicted output
    eEKF(k, :) = yMeas(k, :) - vehicleMeasurementFcn(ekf.State); % ukf.State is x[k|k-1] at this point
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    [xCorrectedEKF(k,:), PCorrectedEKF(k,:,:)] = correct(ekf,yMeas(k, :));
    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    predict(ekf);
end

% Get results for report
%% Figure 1 UKF and EKF estimate comparison
figure();
subplot(4,1,1);
hold on
plot(timeVector, xTrue(:, 1), '--');
plot(timeVector, xCorrectedEKF(:, 1), '-');
legend('True','EKF estimate for x_1')
xlabel('Time [s]');
ylabel('x_1 in km');
hold off

subplot(4,1,2);
plot(timeVector, xTrue(:, 3), '--', timeVector, xCorrectedEKF(:,3), '-');
legend('True','EKF estimate for x_3')
xlabel('Time [s]');
ylabel('x_3 in km/s');

subplot(4,1,3);
plot(timeVector, xTrue(:, 1), '--', timeVector, xCorrectedUKF(:, 1), '-');
legend('True','UKF estimate for x_1')
xlabel('Time [s]');
ylabel('x_1 in km');

subplot(4,1,4);
plot(timeVector, xTrue(:, 3), '--', timeVector, xCorrectedUKF(:,3), '-');
legend('True','UKF estimate for x_3')
xlabel('Time [s]');
ylabel('x_3 in km/s');

%% Figure 2 UKF and EKF covariance comparison
figure();
subplot(3,1,1);
semilogy(timeVector, PCorrectedEKF(:,1,1),'-', ... % 1-sigma upper-bound
    timeVector, PCorrectedUKF(:,1,1), '--');   % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('(km^2)');
title('Error Variance x_1');
legend('EKF', 'UKF');
subplot(3,1,2);
semilogy(timeVector, PCorrectedEKF(:,3,3),'-', ... % 1-sigma upper-bound
    timeVector, PCorrectedUKF(:,3,3), '--');   % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('(km^2/s^2)');
title('Error Variance x_3');
legend('EKF', 'UKF');
subplot(3,1,3);
semilogy(timeVector, PCorrectedEKF(:,5,5),'-', ... % 1-sigma upper-bound
    timeVector, PCorrectedUKF(:,5,5), '--');   % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('no-unit');
title('Error Variance x_5');
legend('EKF', 'UKF');

