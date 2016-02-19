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
syms c real

q = [x; y; th];
dq = [dx; dy; dth];
line = [a,c];

%% ----------------------------------------------------------
%   DEFINE SYSTEM KINEMATICS
% -----------------------------------------------------------

% Bottom of box surface position
Pbtm = [x-w/2; y-h/2];

% CoM position
Pcom = [x; y];

% Cotnact points
Pc1 = Pcom + h/2*[-sin(th);-cos(th)] + w/2*[-cos(th);sin(th)];
Pc2 = Pcom - h/2*[sin(th);cos(th)];
Pc3 = Pcom + h/2*[sin(th);cos(th)] + w/2*[cos(th);-sin(th)];

% CoM velcoity
Vcom= jacobian(Pcom,q)*dq;

% Bottom of box surface velocity
Vbtm = jacobian(Pcom,q)*dq;

% Contact penetration distance
phi1 = pt2line_dist(Pc1,line);
phi2 = pt2line_dist(Pc2,line);
phi3 = pt2line_dist(Pc3,line);
phi1s = y - h/2;
phi2s = y - h/2;
phi3s = y - h/2;

%Phi = [phi1;phi2;phi3];
Phi = [phi1s; phi2s; phi3s];

J = jacobian(Phi,q);

Psi = J*dq;

%% ---------------------------------------------------------
%   DERIVE SYSTEM DYNAMICS
% ----------------------------------------------------------
% KINETIC ENERGY KE_i = 0.5*m_i*Vc_i^2 + 0.5*I_i dq_i^2
KE = 0.5*m*Vcom.'*Vcom + 0.5*I*dth^2;
KE = simplify(KE);

% POTENTIAL ENERGY PE_i = m_i*g*Yc_i;
PE = m*g*Pcom(2);
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
Phi_0 = [];
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
params_list = {'w','h',...
               'm','I',...
               'g'};
           
q_list = {'x','y','th'};
dq_list = {'dx','dy','dth'};
u_list = {'u1'};
line_coeff = {'a','c'};

matlabFunction(D_mtx,C_mtx, G_vec, B_mtx,'File',fullfile(autoFolderName,'autogen_fallingBox_EOM_mtxs.m'),...
   'vars',horzcat(q_list,dq_list,params_list));

% Generate Energy
% ----------------
matlabFunction(KE,PE, 'File', fullfile(autoFolderName,'autogen_fallingBox_Energy.m'),...
               'vars', horzcat(q_list,dq_list,params_list),...
               'outputs',{'U','T'});

% Generate Kinematics
% --------------------
matlabFunction(Pc1,Pc2,Pc3,'File',fullfile(autoFolderName,'autogen_contact_pts.m'),...
                'vars',horzcat(q_list,params_list),...
                'outputs',{'pc1','pc2','pc3'});
           
matlabFunction(Phi,Psi,J,'File',fullfile(autoFolderName,'autogen_contactDyn.m'),...
                'vars',horzcat(q_list,dq_list,params_list,line_coeff),...
                'outputs',{'Phi','Psi','J'});






