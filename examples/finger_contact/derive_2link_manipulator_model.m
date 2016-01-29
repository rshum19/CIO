% FILENAME: derive_3link_invPend_model.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 1/24/2016
%
% DESCRIPTION:
% This script will derive the equation of motion using the 
% Lagrian Dynamics method of a 3DoF Planar Inverted Pendulum
% model of a leg in floating coordiantes.
% The model is written in the form:
%       D(q)ddq + C(q,dq)dq + G(q) = B(q)u + J(q)'lambda
%
% Refer to Apendix B.4.5 in [1] for derivation.
% 
% Generalized coordinates
%   q = [q1 q2 q3 x y];
%
% Notation: 
%   q1 = foot absolute angle
%   q2 = knee absolute angle
%   q3 = hip absolute angle
%   x = x-coordiante of foot position
%   y = y-coordinate of foot position
%
% References:
% [1] Westervelt, Eric R., et al. Feedback control of dynamic 
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

%  Absolute joint angles and velocities
syms q1 dq1 real
syms q2 dq2 real
syms q3 dq3 real 

% Individual link length
syms l1 d1 real
syms l2 d2 real
syms l3 d3 real 

% Individual link masses
syms m1 m2 m3 real

% Links inertias
syms I1 real
syms I2 real
syms I3 real

% Foot position
syms x real
syms y real

% Foot linear velocity
syms dx real
syms dy real

% Input
syms u1 real
syms u2 real


q = [q1; q2; q3];
dq = [dq1; dq2; dq3];
u = [u1; u2];

%% ----------------------------------------------------------
%   DEFINE SYSTEM KINEMATICS
% -----------------------------------------------------------

% Link distal ends positions
P1 = [x + l1*sin(q1);...
       y + l1*cos(q1)];
P2 = [x + l1*sin(q1) + l2*sin(q2);...
       y + l1*cos(q1) + l2*cos(q2)];
P3 = [x + l1*sin(q1) + l2*sin(q2) + l3*sin(q3);...
       y + l1*cos(q1) + l2*cos(q2) + l3*cos(q3)];

% Link center of mass positions
Pc1 = [x + d1*sin(q1);...
       y + d1*cos(q1)];
Pc2 = [x + l1*sin(q1) + d2*sin(q2);...
       y + l1*cos(q1) + d2*cos(q2)];
Pc3 = [x + l1*sin(q1) + l2*sin(q2) + d3*sin(q3);...
       y + l1*cos(q1) + l2*cos(q2) + d3*cos(q3)];
   
% Center of mass position in cartesian coordinates
Pcom = (m1*Pc1 + m2*Pc2 + m3*Pc3)./(m1 + m2 + m3);

%% ----------------------------------------------------------
%   DERIVE SYSTEM DYNAMICS
% -----------------------------------------------------------

% Link distal end linear velocity
V1 = jacobian(P1,q)*dq;
V2 = jacobian(P2,q)*dq;
V3 = jacobian(P3,q)*dq;

% Link center of mass linear velocity
Vc1 = jacobian(Pc1,q)*dq;
Vc2 = jacobian(Pc2,q)*dq;
Vc3 = jacobian(Pc3,q)*dq;

% KINETIC ENERGY KE_i = 0.5*m_i*Vc_i^2 + 0.5*I_i dq_i^2
KE1 = 0.5*m1*Vc1.'*Vc1 + 0.5*I1*dq1^2;
KE1 = simplify(KE1);
KE2 = 0.5*m2*Vc2.'*Vc2 + 0.5*I2*dq2^2;
KE2 = simplify(KE2);
KE3 = 0.5*m3*Vc3.'*Vc3 + 0.5*I3*dq3^2;
KE3 = simplify(KE3);

% Total kinetic energy
KE = KE1 + KE2 + KE3;
KE = simplify(KE);


% POTENTIAL ENERGY PE_i = m_i*g*Yc_i;
PE1 = m1*g*Pc1(2);
PE1 = simplify(PE1);
PE2 = m2*g*Pc2(2);
PE2 = simplify(PE2);
PE3 = m3*g*Pc3(2);
PE3 = simplify(PE3);

% Total potential energy
PE = PE1 + PE2 + PE3;
PE = simplify(PE);

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
Gamma_0 = [q2; q3];
B_mtx = simplify(jacobian(Gamma_0,q));
B_mtx = B_mtx';

% Pontential contact location vector
Phi_vec = [P1(2); P2(2); P3(2); y];

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
params_list = {'l1','l2','l3',...
               'd1','d2','d3',...
               'm1','m2','m3',...
               'I1','I2','I3',...
               'g'};

q_list = {'q1','q2','q3','x','y'};
dq_list = {'dq1','dq2','dq3','dx','dy'};
u_list = {'u2','u3'};

matlabFunction(D_mtx,C_mtx, G_vec, B_mtx,Phi_vec, J_mtx,'File',fullfile(autoFolderName,'invPend_3link_EOM_mtxs.m'),...
   'vars',horzcat(q_list,dq_list,u_list,params_list));

% Generate Energy
% ----------------
matlabFunction(KE,PE, 'File', fullfile(autoFolderName,'invPend_3link_Energy.m'),...
               'vars', horzcat(q_list,dq_list,params_list),...
               'outputs',{'U','T'});

% Generate Kinematics
% --------------------
params_list = {'l1','l2','l3'};

matlabFunction(P1,P2,P3,V1,V2,V3,'File',fullfile(autoFolderName,'invPend_3link_kin.m'),...
                'vars',horzcat(q_list,dq_list,params_list),...
                'outputs',{'p1','p2','p3','dp1','dp2','dp3'});
            
