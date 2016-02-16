function [params] = params_invPend_2DoF
% params parametrs

%
%
% Parameters based on robot described in:
% [1] 
%
%---- ENVIRONMENT
g = 9.81;       % Gravity [m/s^2]

params.g = g;

%---- MASS
m1 = 1;
m2 = 1;
params.m1 = m1;  % [kg]
params.m2 = m2;  % [kg]

%---- GEOMETRY
% Link length
l1 = 0.5;
l2 = 0.5;
params.l1 = l1;    % [m]
params.l2 = l2;    % [m]

%---- MOMENT OF INERTIA
params.I1 = (m1*l1.^2)/12;
params.I2 = (m2*l2.^2)/12;

% COM distance from joint
params.d1 = 0.25;
params.d2 = 0.25;

end

