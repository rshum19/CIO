%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
clc; clear; close all;

addpath('helperFncs');
%% ----------------------------------------------------------
%   MODEL VARIABLES
% -----------------------------------------------------------

% Environment 
syms g real         % gravity

% Box mass
syms m real
syms I real

% Position,Velocity,Acceleration coordinates
syms x dx ddx real
syms y dy ddy real
syms th dth ddth real

% Box dimensions 
syms w real
syms h real

% Ground line 
syms a real
syms b real
syms c real

q = [x; y; th];
dq = [dx; dy; dth];
line = [a,b,c];

%% ----------------------------------------------------------
%   DEFINE SYSTEM KINEMATICS
% -----------------------------------------------------------

% Bottom of box surface position
Pbtm = [x-w/2; y-h/2];

% CoM position
Pcom = [x; y];

% Cotnact points
Pc1 = Pcom + h/2*[-sin(th);-cos(th)] + w/2*[-cos(th);sin(th)];
Pc2 = Pcom + h/2*[-sin(th);-cos(th)];
Pc3 = Pcom + h/2*[sin(th);cos(th)] + w/2*[cos(th);-sin(th)];

% CoM velcoity
Vcom= jacobian(Pcom,q)*dq;

% Bottom of box surface velocity
Vbtm = jacobian(Pcom,q)*dq;

% Contact penetration distance
phi1 = pt2line_dist(Pc1,line);
phi2 = pt2line_dist(Pc2,line);
phi3 = pt2line_dist(Pc3,line);

Phi = [phi1;phi2;phi3];

J = jacobian(Phi,q);

Psi = J*dq;

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

%% ----------------------------------------------------------
%   GENERATE MATLAB FUNCTIONS
% -----------------------------------------------------------
% Create folder to save autogerated functions
autoFolderName = 'autogenFncs';
if ~exist(autoFolderName,'dir')
  mkdir(autoFolderName);
end

% Generate Dynamics
% -----------------
params_list = {'w','h',...
               'm','I',...
               'g'};
           
q_list = {'x','y','th'};
dq_list = {'dq1','dq2'};
u_list = {'u1'};
line_coeff = {'a','b','c'};

% Generate Kinematics
% --------------------
matlabFunction(Pc1,Pc2,Pc3,'File',fullfile(autoFolderName,'autogen_contact_pts.m'),...
                'vars',horzcat(q_list,params_list),...
                'outputs',{'pc1','pc2','pc3'});
           
matlabFunction(Phi,Psi,J,'File',fullfile(autoFolderName,'autogen_contactDyn.m'),...
                'vars',horzcat(q_list,params_list,line_coeff),...
                'outputs',{'pc1','pc2','pc3'});






