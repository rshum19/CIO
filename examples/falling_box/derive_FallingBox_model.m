%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
clc; clear; close all;

%% ----------------------------------------------------------
%   MODEL VARIABLES
% -----------------------------------------------------------

% Environment 
syms g real         % gravity

% Box mass
syms m real

% Position,Velocity,Acceleration coordinates
syms x dx ddx real
syms y dy ddy real

% Box dimensions 
syms w real
syms h real

q = [x; y];
dq = [dx; dy];

%% ----------------------------------------------------------
%   DEFINE SYSTEM KINEMATICS
% -----------------------------------------------------------

% Bottom of box surface position
Pbtm = [x-w/2; y-h/2];

% CoM position
Pcom = [x; y];

% CoM velcoity
Vcom= jacobian(Pcom,q)*dq;

% Bottom of box surface velocity
Vbtm = jacobian(Pcom,q)*dq;

Phi = q(2) - h/2;

J = jacobian(Phi,q);

Phi = J*dq;


%% ---------------------------------------------------------
%   DERIVE SYSTEM DYNAMICS
% ----------------------------------------------------------

% Sum of vertical component forces 
%  Ma_y = -Fg = -M*g
%   a_y = -g
%
% Sum of horizontal component forces
%  Ma_x = 0
%   a_y = 0










