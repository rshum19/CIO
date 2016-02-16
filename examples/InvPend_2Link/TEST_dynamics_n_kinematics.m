%INVPEND_DYNAMICS wrapper to numerically compute 3-link invertered Pendulum's EOMs
% FILENAME: invPend_Dynamics.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%
% DESCRIPTION:
%
addpath('../../')
%% ----------------------------------------------------------
%   DEFINE ODE PROBLEM
% -----------------------------------------------------------
% Initialize model
params = params_invPend_2DoF;

% Integrator 
t0 = 0;
tF = 10;
tspan = [t0,tF];

% Initial Condition
q0 = [0; 0.5];
dq0 = zeros(2,1);
z0 = [q0;dq0]';

%% ----------------------------------------------------------
%   SOLVE
% -----------------------------------------------------------
u = [];
[Tsol,Xsol] = ode45(@(t,z)invPend_Dynamics(t,z,u,params),tspan,z0);
Xsol = Xsol';

%% ----------------------------------------------------------
%   DISPLAY RESULTS
% -----------------------------------------------------------

% Plots
figure
subplot(2,1,1)
plot(Tsol,Xsol(1:2,:));
legend('joint 1','joint 2')
xlabel('Time [sec]');
ylabel('Angle [rad]');

subplot(2,1,2)
plot(Tsol,Xsol(3:4,:));
legend('joint 1','joint 2')
xlabel('Time [sec]');
ylabel('Angle rate [rad/sec]');

% Tip trajectory
p1Vec = [];
p2Vec = [];
for i = 1:length(Tsol)
    z = Xsol(:,i);
    t = Tsol(i);
    [p1,p2] = invPend_Kinematics(t,z,params);
    p1Vec = [p1Vec, p1];
    p2Vec = [p2Vec, p2];
end
figure
plot(p2Vec(1,:),p2Vec(2,:))
title('Tip Trajectory');

% Animate the results:
A.plotFunc = @(t,z)( drawInvPend2(t,z,params) );
A.speed = 0.25;
A.figNum = 101;
animate(Tsol,Xsol,A)



