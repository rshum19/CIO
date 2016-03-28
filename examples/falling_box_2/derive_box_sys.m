% ----------------------------------------------------------
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
syms I real

% Position,Velocity,Acceleration coordinates
syms x dx ddx real
syms y dy ddy real
syms th dth ddth real
syms l1 l2 l3 real

% Box dimensions 
syms w real
syms h real

% Ground line 
syms a real
syms c real

q = [x; y; th];
dq = [dx; dy; dth];
line = [a,c];
lambda = [l1;l2;l3];

%% ----------------------------------------------------------
%   DEFINE SYSTEM KINEMATICS
% -----------------------------------------------------------

% CoM position
Pcom = [x; y];

% Cotnact points
R = [cos(th), -sin(th); sin(th), cos(th)];
Pc1 = [x; y-h/2];
Pc2 = [x-w/2; y-h/2];
Pc3 = [x+w/2;y-h/2];
Pc1 = Pcom + R*(Pc1-Pcom);
Pc2 = Pcom + R*(Pc2-Pcom);
Pc3 = Pcom + R*(Pc3-Pcom);
% CoM velcoity
Vcom= jacobian(Pcom,q)*dq;

% Contact penetration distance
v1 = [1;0];
v2 = [0;0];
Phi1 =((v2(1)-v1(1))*(v1(2)-Pc1(2)) - (v1(1)-Pc1(1))*(v2(2)-v1(2)))/norm(v2-v1);
Phi1 = simplify(Phi1);

Phi2 =((v2(1)-v1(1))*(v1(2)-Pc2(2)) - (v1(1)-Pc2(1))*(v2(2)-v1(2)))/norm(v2-v1);
Phi2 = simplify(Phi2);

Phi3 =((v2(1)-v1(1))*(v1(2)-Pc3(2)) - (v1(1)-Pc3(1))*(v2(2)-v1(2)))/norm(v2-v1);
Phi3 = simplify(Phi3);

Phi = [Phi1;Phi2;Phi3];
% Projection of constraint forces to generalized coordinates
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

% ddq
ddq = D_mtx\(- G_vec + J'*lambda);
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
lambda_list = {'l1','l2','l3'};

matlabFunction(D_mtx,C_mtx, G_vec, B_mtx,'File',fullfile(autoFolderName,'autogen_fallingBox_EOM_mtxs.m'),...
   'vars',horzcat(q_list,dq_list,params_list));

matlabFunction(ddq,'File',fullfile(autoFolderName,'autogen_fallingBox_ddq.m'),...
   'vars',horzcat(q_list,dq_list,lambda_list,params_list));

% Generate Energy
% ----------------
matlabFunction(KE,PE, 'File', fullfile(autoFolderName,'autogen_fallingBox_Energy.m'),...
               'vars', horzcat(q_list,dq_list,params_list),...
               'outputs',{'U','T'});

% Generate Kinematics
% --------------------
matlabFunction(Pc1,'File',fullfile(autoFolderName,'autogen_contact_pts.m'),...
                'vars',horzcat(q_list,params_list),...
                'outputs',{'pc1'});
           
matlabFunction(Phi,Psi,J,'File',fullfile(autoFolderName,'autogen_contactDyn.m'),...
                'vars',horzcat(q_list,dq_list,params_list),...
                'outputs',{'Phi','Psi','J'});






