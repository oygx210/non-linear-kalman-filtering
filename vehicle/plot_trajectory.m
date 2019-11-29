%% Plot Spacecraft trajectory for report
% True Initial state: xhat[k|k-1]
initialState = [6500.4; 349.14; -1.8093; -6.7967; 0];


% Get true trajectory for x[1:4] noise-less measurements
% ODE update rate is every 100ms
T = 0.05; % [s] Filter sample time
% Siumlate for a time of 200s
timeVector = 0:T:200;
% Get true noiseless samples
[~, xTrue]=ode45(@vehicleStateFcnContinuous2,timeVector,initialState);
%Plot them
figure()
% Plot sensor location as big hollow circle
xr = 6370;
yr = 0;
hold on
plot(xr, yr, 'o', 'MarkerSize', 5, 'color', 'k')
plot(xTrue(:, 1), xTrue(:, 2), '-', 'color', 'k')

% Plot Earth Surface
circr = @(radius,rad_ang)  [radius*cos(rad_ang);  radius*sin(rad_ang)];         % Circle Function For Angles In Radians
circd = @(radius,deg_ang)  [radius*cosd(deg_ang);  radius*sind(deg_ang)];       % Circle Function For Angles In Degrees
N = 100;                                                         % Number Of Points In Complete Circle
r_angl = linspace(pi/3, -pi/6, N);                             % Angle Defining Arc Segment (radians)
radius = 6370;                                                   % Arc Radius
xy_r = circr(radius,r_angl);                                    % Matrix (2xN) Of (x,y) Coordinates
plot(xy_r(1,:), xy_r(2,:), '--', 'color', 'k')                                % Draw An Arc Of Blue Stars

ylim([-200 500])
xlim([6350 6500])
xlabel('x_1 (km)');
ylabel('x_2 (km)');
hold off
