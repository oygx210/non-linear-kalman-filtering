function yk = vehicleMeasurementFcn2(xk)
% vehicleMeasurementFcn measurement function for discrete
% time nonlinear state estimator
%
% yk = vehicleMeasurementFcn(xk)
%
% Inputs:
%    xk - x[k], states at time k
%
% Outputs:
%    yk - y[k], measurements at time k
%
xr = 6370;
yr = 0;
yk1 = (sqrt((xk(:, 1) - xr).^2 + (xk(:, 2) - yr).^2));
yk2 = atan((xk(:, 2) - yr)./(xk(:, 1) - xr));
yk = [yk1 yk2];
end
