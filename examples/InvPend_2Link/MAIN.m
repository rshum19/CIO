% PURPOSE:  
% FILENAME: .m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%------------------- Instructions ---------------------------

%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
% Clear workspace
clc; clear;

% Add paths
addpath('wrapperFncs/');
addpath('autogenFncs/');
addpath(genpath('../../'));
%% ----------------------------------------------------------
%   MODEL PROPERTIES
%   Intialize your dynamic model parameter values and dyanmics
% -----------------------------------------------------------
OCP.model.params = params_invPend_2DoF;
OCP.model.dynamics = @(t,x,u)invPend_Dynamics(t,x,u,OCP.model.params);

%% ----------------------------------------------------------
%   DEFINE OCP PROBLEM PROPERTIES
% -----------------------------------------------------------

% COST/OBJECTIVE FUNCTIONS
% ------------------------
%OCP.costFnc = @(t,x,u)invPend_CostFnc(t,x,u, OCP.model.params);
% Need to change to have the above format
OCP.pathCostFnc = @(t,x,u)(u.^2);
OCP.bndCostFnc = [];

% INITIAL GUESS
% ------------------------
% Set the type of initial guess:
%   linear: Just define a intial and final state and OptCtrlSolver will
%           make a linear interpolation between the two points
%   custom: The user provides a custom initial guess "shape". 
%           NOTE: custom initial guess has to be descritized with the same
%           size than OCP.options.nGrid
OCP.options.IGtype = 'linear';

% Time span
t0 = 0;
tF = 2.5;
OCP.ig.time = [t0,tF];

% State 
stepAngle = 0.2;
stepRate = (2*stepAngle)/(tF-t0);
x0 = [-stepAngle; stepAngle; stepRate; -stepRate];
xF = [stepAngle; -stepAngle; stepRate; -stepRate];
OCP.ig.state = [x0,xF];

% Control
u0 = 0;
uF = 0;
OCP.ig.control = [u0,uF];

% CONSTRAINTS & BOUNDARIES
% ------------------------
 
% Nonlinear constraints
%OCP.pathCst = @(t,x,u)pathCst(z);
%OCP.bndCst = @(t0,x0,u0,tF,xF,uF)bndCst(z);
OCP.pathCst = [];
OCP.bndCst = [];

% You can let time to be free by not setting any bounds on the final time
OCP.bounds.initTime.lb = t0;
OCP.bounds.initTime.ub = t0;
OCP.bounds.finalTime.lb = tF;
OCP.bounds.finalTime.ub = tF;

% State:
%   [q1; q2; dq1; dq2];
%  
OCP.bounds.state.lb = [-2*pi; -2*pi; -inf(2,1)]; % Change to so they are set from the model parameters
OCP.bounds.state.ub = [2*pi; 2*pi; inf(2,1)];
OCP.bounds.initState.lb = [pi; pi; 0; 0];
OCP.bounds.initState.ub = [pi; pi; 0; 0];
OCP.bounds.finalState.lb = zeros(4,1);
OCP.bounds.finalState.ub = zeros(4,1);

maxTorque = 25; 
OCP.bounds.control.lb = -maxTorque; % Change so they are set from model parametes
OCP.bounds.control.ub = maxTorque;

%% ----------------------------------------------------------
%   SOLVER OPTIONS
% -----------------------------------------------------------

method = 'euler';
% method = 'trapezoidal';
% method = 'hermiteSimpson';

OCP.options.method = method;
OCP.options.nGrid = 15;

% For a full list of options refer to :
%   http://www.mathworks.com/help/optim/ug/fmincon.html#inputarg_options

OCP.options.fminOpt.MaxFunEval = 1e5;

%% ----------------------------------------------------------
%   SOLVE NLP PROBLEM
% -----------------------------------------------------------

soln = OptCtrlSolver(OCP);

tGrid = soln(end).grid.time;
t = linspace(tGrid(1),tGrid(end),100);
z = soln(end).interp.state(t);
u = soln(end).interp.control(t);


%% ----------------------------------------------------------
%   PLOT RESULTS
% -----------------------------------------------------------
dyn = OCP.model.params;

figure
subplot(3,1,1)
plot(t,z(1:2,:));
legend('joint 1','joint 2')
xlabel('Time [sec]');
ylabel('Angle [rad]');

subplot(3,1,2)
plot(t,z(3:4,:));
legend('joint 1','joint 2')
xlabel('Time [sec]');
ylabel('Angle rate [rad/sec]');

subplot(3,1,3)
plot(t,u)
xlabel('Time [sec]');
ylabel('U control input');

% Animate the results:
A.plotFunc = @(t,z)( drawInvPend2(t,z,dyn) );
A.speed = 0.25;
A.figNum = 101;
animate(t,z,A)

%%% TODO
% Draw a stop-action animation:
