%% EKF Part
% Your initial state guess at time k, utilizing measurements up to time k-1: xhat[k|k-1]
initialStateGuess = [2;0]; % xhat[k|k-1]
% Construct the filter
ekf = extendedKalmanFilter(...
    @vdpModStateFcn,... % State transition function
    @vdpMeasurementNonAdditiveNoiseFcn2,... % Measurement function
    initialStateGuess,...
    'HasAdditiveMeasurementNoise',false);

R = 0.2; % Variance of the measurement noise v[k]
ekf.MeasurementNoise = R;

ekf.ProcessNoise = diag([0.02 0.1]);

T = 0.05; % [s] Filter sample time
timeVector = 0:T:5;
[~,xTrue]=ode45(@vdpModStateFcnContinuous2,timeVector,[2;0]);

rng(1); % Fix the random number generator for reproducible results
yTrue = xTrue(:,1);
yMeas = yTrue .* (1+sqrt(R)*randn(size(yTrue))); % sqrt(R): Standard deviation of noise

Nsteps = numel(yMeas); % Number of time steps
xCorrectedEKF = zeros(Nsteps,2); % Corrected state estimates
PCorrectedEKF = zeros(Nsteps,2,2); % Corrected state estimation error covariances
eEKF = zeros(Nsteps,1); % Residuals (or innovations)

for k=1:Nsteps
    % Let k denote the current time.
    %
    % Residuals (or innovations): Measured output - Predicted output
    eEKF(k) = yMeas(k) - vdpMeasurementFcn(ekf.State); % ukf.State is x[k|k-1] at this point
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    [xCorrectedEKF(k,:), PCorrectedEKF(k,:,:)] = correct(ekf,yMeas(k));
    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    predict(ekf);
end

%% UKF Part
% Your initial state guess at time k, utilizing measurements up to time k-1: xhat[k|k-1]
initialStateGuess = [2;0]; % xhat[k|k-1]
% Construct the filter
ukf = unscentedKalmanFilter(...
    @vdpModStateFcn,... % State transition function
    @vdpMeasurementNonAdditiveNoiseFcn2,... % Measurement function
    initialStateGuess,...
    'HasAdditiveMeasurementNoise',false);

R = 0.2; % Variance of the measurement noise v[k]
ukf.MeasurementNoise = R;

ukf.ProcessNoise = diag([0.02 0.1]);

T = 0.05; % [s] Filter sample time
timeVector = 0:T:5;
[~,xTrue]=ode45(@vdpModStateFcnContinuous2,timeVector,[2;0]);

rng(1); % Fix the random number generator for reproducible results
yTrue = xTrue(:,1);
yMeas = yTrue .* (1+sqrt(R)*randn(size(yTrue))); % sqrt(R): Standard deviation of noise

Nsteps = numel(yMeas); % Number of time steps
xCorrectedUKF = zeros(Nsteps,2); % Corrected state estimates
PCorrectedUKF = zeros(Nsteps,2,2); % Corrected state estimation error covariances
eUKF = zeros(Nsteps,1); % Residuals (or innovations)

for k=1:Nsteps
    % Let k denote the current time.
    %
    % Residuals (or innovations): Measured output - Predicted output
    eUKF(k) = yMeas(k) - vdpMeasurementFcn(ukf.State); % ukf.State is x[k|k-1] at this point
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    [xCorrectedUKF(k,:), PCorrectedUKF(k,:,:)] = correct(ukf,yMeas(k));
    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    predict(ukf);
end

%% Plot 1 - State tracking of UKF vs. EKF
figure();
subplot(2,1,1);
plot(timeVector,xTrue(:,1),timeVector,xCorrectedEKF(:,1), '-', timeVector, xCorrectedUKF(:,1), '-o',...
    timeVector, yMeas(:),'--');
legend('True','EKF estimate', 'UKF estimate', 'Measured')
ylim([-2.6 2.6]);
ylabel('x_1');
subplot(2,1,2);
plot(timeVector,xTrue(:,2),timeVector,xCorrectedEKF(:,2), '-', timeVector, xCorrectedUKF(:,2), '-o');
legend('True','EKF estimate', 'UKF estimate')
ylim([-3 1.5]);
xlabel('Time [s]');
ylabel('x_2');

%% Plot 2 - State tracking of UKF vs. EKF
figure();
subplot(2,1,1);
mseEKF = (xTrue(:,1) - xCorrectedEKF(:,1)).^2;
mseUKF = (xTrue(:,1) - xCorrectedUKF(:,1)).^2;
plot(timeVector, mseEKF, '-', timeVector, mseUKF, '--');
legend('EKF mse', 'UKF mse')
ylim([-2.6 2.6]);
ylabel('mse in x_1');
subplot(2,1,2);
mseEKF = (xTrue(:,2) - xCorrectedEKF(:,2)).^2;
mseUKF = (xTrue(:,2) - xCorrectedUKF(:,2)).^2;
plot(timeVector, mseEKF, '-', timeVector, mseUKF, '--');
legend('EKF mse', 'UKF mse')
ylim([-2.6 2.6]);
ylabel('mse in x_2');
