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
syms q1 dq1 ddq1 real
syms q2 dq2 ddq2 real

% Individual link length
syms l1 d1 real
syms l2 d2 real

% Individual link masses
syms m1 real
syms m2 real

% Links inertias
syms I1 real
syms I2 real

% Foot position
syms xb real
syms yb real

% Input
syms u1 real
%syms u2 real

% Relative angles
syms q1L real   % q1L = q1
syms q2L real   % q2L = q1-q2

q = [q1; q2];
qL = [q1L; q2L];
dq = [dq1; dq2];
ddq = [ddq1; ddq2];
u = [u1];

%% ----------------------------------------------------------
%   DEFINE SYSTEM KINEMATICS
% -----------------------------------------------------------

% Link distal ends positions
P1 = [l1*sin(q1);...
       l1*cos(q1)];
P2 = [l1*sin(q1) + l2*sin(q2);...
       l1*cos(q1) + l2*cos(q2)];

% Link center of mass positions
Pc1 = [d1*sin(q1);...
       d1*cos(q1)];
Pc2 = [l1*sin(q1) + d2*sin(q2);...
       l1*cos(q1) + d2*cos(q2)];
   
% Center of mass position
Pcom = (m1*Pc1 + m2*Pc2)./(m1 + m2);

% Link distal end linear velocity
V1 = jacobian(P1,q)*dq;
V2 = jacobian(P2,q)*dq;

% Link center of mass linear velocity
Vc1 = jacobian(Pc1,q)*dq;
Vc2 = jacobian(Pc2,q)*dq;

%% ----------------------------------------------------------
%   DERIVE SYSTEM DYNAMICS
% -----------------------------------------------------------

% KINETIC ENERGY KE_i = 0.5*m_i*Vc_i^2 + 0.5*I_i dq_i^2
KE1 = 0.5*m1*Vc1.'*Vc1 + 0.5*I1*dq1^2;
KE1 = simplify(KE1);
KE2 = 0.5*m2*Vc2.'*Vc2 + 0.5*I2*dq2^2;
KE2 = simplify(KE2);

% Total kinetic energy
KE = KE1 + KE2;
KE = simplify(KE);

% POTENTIAL ENERGY PE_i = m_i*g*Yc_i;
PE1 = m1*g*Pc1(2);
PE1 = simplify(PE1);
PE2 = m2*g*Pc2(2);
PE2 = simplify(PE2);

% Total potential energy
PE = PE1 + PE2;
PE = simplify(PE);

% COMPUTING EOM MATRICIES
% -----------------------
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
% assumes you have ankle torques
Phi_0 = [q2];
B_mtx = simplify(jacobian(Phi_0,q));
B_mtx = B_mtx';
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
params_list = {'l1','l2',...
               'd1','d2',...
               'm1','m2',...
               'I1','I2',...
               'g'};

q_list = {'q1','q2'};
dq_list = {'dq1','dq2'};
u_list = {'u1'};

matlabFunction(D_mtx,C_mtx, G_vec, B_mtx,'File',fullfile(autoFolderName,'autogen_invPend_2DoF_EOM_mtxs.m'),...
   'vars',horzcat(q_list,dq_list,u_list,params_list));

% Generate Energy
% ----------------
matlabFunction(KE,PE, 'File', fullfile(autoFolderName,'autogen_invPend_2DoF_Energy.m'),...
               'vars', horzcat(q_list,dq_list,params_list),...
               'outputs',{'U','T'});

% Generate Kinematics
% --------------------
params_list = {'l1','l2'};

matlabFunction(P1,P2,V1,V2,'File',fullfile(autoFolderName,'autogen_invPend_2DoF_kin.m'),...
                'vars',horzcat(q_list,dq_list,params_list),...
                'outputs',{'p1','p2','dp1','dp2'});
            

