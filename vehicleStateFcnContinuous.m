function dxdt = vehicleStateFcnContinuous(x)
%Evaluate the ODE associated with the state vector x[1:4]
% Distance from centre of earth
R = sqrt(x(1)^2 + x(2)^2);
% Current velocity
V = sqrt(x(3)^2 + x(4)^2);
% Some parameter
beta_o = 0.59783;
beta = beta_o;
Ho = 13.406;
Gmo = 3.9860e5;
Ro = 6374;
% Drag force 
D = -beta*exp((Ro - R)/Ho)*V;
% Gravitationoal force
G = -Gmo/R^3;
dxdt = [x(3); x(4); D*x(3) + G*x(1); D*x(4) + G*x(2)];
end
