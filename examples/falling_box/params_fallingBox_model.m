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
w = 4;       %[m]
h = 4;

% Link length
params.w = w;
params.h = h;

%---- MASS
m = 5;         % [kg]
params.m = m;

%---- MOMENT OF INERTIA
params.I = m/12*(h^2 + w^2);

%---- Other
params.line = [0;0];
end

