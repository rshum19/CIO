%
% FILENAME: .m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%
% DESCRIPTION:
%

%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
clc; clear; close all;

%% ----------------------------------------------------------
%   DEFINE ODE PROBLEM
% -----------------------------------------------------------
% Initialize model
params = params_fallingBox_model;

% Integrator 
t0 = 0;
tF = 2;
tspan = [t0,tF];

% Initial Condition
x0 = 0;
y0 = 5;

q0 = [x0;y0];
dq0 = zeros(2,1);
z0 = [q0;dq0];


%% ----------------------------------------------------------
%   SOLVE
% -----------------------------------------------------------
[Tsol,Xsol] = ode45(@(t,z)fallingBox_dynamics(t,z,[],params),tspan,z0);
Xsol = Xsol';

%% ----------------------------------------------------------
%   DISPLAY RESULTS
% -----------------------------------------------------------

% Plots
figure
subplot(1,2,1)
plot(Tsol,Xsol(1,:))
xlabel('Time [sec]')
ylabel('X pos [m]')

subplot(1,2,2)
plot(Tsol,Xsol(2,:))
xlabel('Time [sec]')
ylabel('Y pos [m]')

figure
plot(Xsol(1,:),Xsol(2,:))
xlabel('X pos [m]')
ylabel('Y pos [m]')

%% ----------------------------------------------------------
%   SAVE RESULTS
% -----------------------------------------------------------
save('fallingBox_data.mat','Tsol','Xsol')
