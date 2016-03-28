function [params] = params_planar_3link
% params parametrs

%---- ENVIRONMENT
g = 9.81;       % Gravity [m/s^2]
mu = 0.7;       % Friction coefficient (0.6 - 0.85)
params.mu = mu; 
params.g = g;

%---- GEOMETRY
l1 = 0.66;       %[m]
l2 = 0.55;
l3 = 0.837;

% Link length
params.l1 = l1;
params.l2 = l2;
params.l3 = l3;

%---- MASS
m1 = 4.55;         % [kg]
m2 = 45;         % [kg]
m3 = 13.85;         % [kg]

params.m1 = m1;
params.m2 = m2;
params.m3 = m3;

%---- MOMENT OF INERTIA
params.I1 = 0.1579;
params.I2 = 1.9155;
params.I3 = 0.9753;

%---- Other

end

