function [params] = params_fallingBox_model
% params parametrs

%
%
%
%---- ENVIRONMENT
g = 9.81;       % Gravity [m/s^2]
mu = 0.7;       % Friction coefficient (0.6 - 0.85)
params.mu = mu; 
params.g = g;

%---- GEOMETRY
w = 0.2;       %[m]
h = 0.2;

% Link length
params.w = w;
params.h = h;

%---- MASS
m = 5;         % [kg]
params.m = m;

%---- MOMENT OF INERTIA
%params.I1 = 0.5*m0*l0^2;
%params.I2 = 0.002*m0*l0^2;
%params.I3 = 0.002*m0*l0^2;

%---- Other

end

