function x = vehicleStateFcn(x)
% vdpStateFcn Discrete-time approximation to van der Pol ODEs for mu = 1. 
% Sample time is 0.05s.
%
% Example state transition function for discrete-time nonlinear state
% estimators.
%
% xk1 = vdpStateFcn(xk)
%
% Inputs:
%    xk - States x[k]
%
% Outputs:
%    xk1 - Propagated states x[k+1]
%
% Euler integration of continuous-time dynamics x'=f(x) with sample time dt
dt = 0.05; % [s] Sample time
x = x + vehicleStateFcnContinuous(x)*dt;
end

