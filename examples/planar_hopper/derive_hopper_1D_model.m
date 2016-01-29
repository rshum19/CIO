% FILENAME: derive_hopper_1D_model.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 1/27/2016
%
% DESCRIPTION:
% This script will derive the equation of motion using the 
% Lagrian Dynamics method of a 1D Planar Running Robot
% model in floating coordiantes. This robot is taken from [1]
% The model is written in the form:
%       D(q)ddq + C(q,dq)dq + G(q) = B(q)u + J(q)'lambda
%
%   D(q) = Mass matrix
%   C(q,dq) = Coriolis forces
%   G(q) = gravitational forces
%   B(q) = output matrix
%   J(q) = contact jacobian
% Refer to Apendix B.4.5 in [2] for derivation.
% 
% Generalized coordinate set
%   q = [x y phi alpha l];
%   u = actuator forces
%   lambda = contact forces

% Notation: 
%   x = x-position of body
%   y = y-position of body
%   phi = body orientation
%   alpha = hip joint angle
%   l = leg length
%
% References:
% [1] C. Remy, K. Buffinton, and R. Siegwart, ?Comparison of cost functions
%     for electrically driven running robots,? 2012.
% [2] Westervelt, Eric R., et al. Feedback control of dynamic 
%     bipedal robot locomotion. Vol. 28. CRC press, 2007.
%
%%%%% TODO
% - Add image of system to folder
%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
clc; clear; close all;

%% ----------------------------------------------------------
%   MODEL VARIABLES
% -----------------------------------------------------------

% Environment 
syms g real         % gravity

% Body position
syms x dx real
syms y dy real
syms phi dphi real

% Hip joint angle
syms alpha dalpha real

% Leg length
syms l dl real

% Individual body masses
syms m1 m2 ml real

%System parameters
syms l0 real
syms k real
syms b real

% Links inertias
syms I1 real
syms I2 real

% Inputs
% Actuator displacement
syms ul dul real
syms taul real

q = [y; l; ul];
dq = [dy; dl; dul];

%% ----------------------------------------------------------
%   DEFINE SYSTEM KINEMATICS
% -----------------------------------------------------------

% Link distal ends positions
P1 = y;
P2 = y-l;

% Link center of mass positions
Pc1 = y;
Pc2 = y-l;

% Center of mass position in cartesian coordinates
Pcom = (m1*Pc1 + m2*Pc2 )./(m1 + m2);

%% ----------------------------------------------------------
%   DERIVE SYSTEM DYNAMICS
% -----------------------------------------------------------

% Link distal end linear velocity
V1 = jacobian(P1,q)*dq;
V2 = jacobian(P2,q)*dq;

% Link center of mass linear velocity
Vc1 = jacobian(Pc1,q)*dq;
Vc2 = jacobian(Pc2,q)*dq;

% KINETIC ENERGY KE_i = 0.5*m_i*Vc_i^2 + 0.5*I_i dq_i^2
KE = 0.5*m1*dy^2 + 0.5*m2*dl^2 + 0.5*ml*dul^2;
KE = simplify(KE);

% POTENTIAL ENERGY PE_i = m_i*g*Yc_i;
PE = m1*g*y + m2*g*(y-l);
PE = simplify(PE);

F = k*(l0 + ul - l) + b*(dul - dl);

% COMPUTING EOM MATRICIES
%-------------------------
% Gravity vector
G_vec = simplify(jacobian(PE,q).');

% Mass-Inertia matrix
D_mtx = simplify(jacobian(KE,dq).');
D_mtx = simplify(jacobian(D_mtx,dq));

% Coriolis and centrifugal matrix
syms C_mtx real
n=max(size(q));
for k=1:n
    for j=1:n
        C_mtx(k,j) = 0*g;
        for i=1:n
            C_mtx(k,j) = C_mtx(k,j)+ 1/2*(diff(D_mtx(k,j),q(i)) + ...
                diff(D_mtx(k,i),q(j)) - ...
                diff(D_mtx(i,j),q(k)))*dq(i);
        end
    end
end
C_mtx = simplify(C_mtx);

% Input matrix  
Gamma_0 = [l; ul];
B_mtx = simplify(jacobian(Gamma_0,q));
B_mtx = B_mtx';

u_vec = [F;taul];

% Pontential contact location vector
Phi_vec = P2;

% Jacobian matrix projecting constraint forces
J_mtx = jacobian(Phi_vec,q);

%% ----------------------------------------------------------
%   GENERATE MATLAB FUNCTIONS
% -----------------------------------------------------------
% Create folder to save autogerated functions
autoFolderName = 'autogenFncs';
if ~exist(autoFolderName,'dir')
  mkdir(autoFolderName);
end

% Generate Dynamics
% ----------------
params_list = {'l0','k','b',...
               'm1','m2','ml',...
               'g'};

q_list = {'y','l','ul'};
dq_list = {'dy','dl','dul'};
u_list = {'taul'};

matlabFunction(D_mtx,C_mtx, G_vec, B_mtx,u_vec,Phi_vec, J_mtx,F,'File',fullfile(autoFolderName,'hopper_1D_EOM_mtxs.m'),...
   'vars',horzcat(q_list,dq_list,u_list,params_list));

% Generate Energy
% ----------------
matlabFunction(KE,PE, 'File', fullfile(autoFolderName,'hopper_1D_Energy.m'),...
               'vars', horzcat(q_list,dq_list,params_list),...
               'outputs',{'U','T'});

% Generate Kinematics
% --------------------
params_list = {'l0'};

matlabFunction(P1,P2,V1,V2,'File',fullfile(autoFolderName,'hopper_1D_kin.m'),...
                'vars',horzcat(q_list,dq_list,params_list),...
                'outputs',{'p1','p2','dp1','dp2'});
            
