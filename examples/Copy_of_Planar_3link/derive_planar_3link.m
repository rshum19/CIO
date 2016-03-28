function derive_planar_3link
% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
clc; clear; close all;

%% ----------------------------------------------------------
%   MODEL VARIABLES
% -----------------------------------------------------------

% Coordinates 
syms xc yc real
syms dxc dyc real
syms x y th real
syms dx dy dth real
syms a1 a2 real
syms da1 da2 real

% Dimension properties
syms l1 l2 l3 real
syms m1 m2 m3 real
syms I1 I2 I3 real

% Forces
syms lamb1 lamb2 lamb3 lamb4 real

% Environment
syms G real

q = [a1; a2; a3];
dq = [dth; da1; da2];
q2 = [x; y; th; a1; a2];
dq2 = [dx; dy; dth; da1; da2];
lambda = [lamb1; lamb2; lamb3; lamb4];

%% ----------------------------------------------------------
%   SYSTEM KINEMATICS
% -----------------------------------------------------------

%--- Positions & Orientation
% Base/reference link (torso)
g2 = [x; y; th];
f2 = g2 + l2/2*[sin(th); cos(th); 0];
h2 = g2 + l2/2*[-sin(th); -cos(th); 0];

% Left link (arm)
h1 = f2 + [0; 0; a1];
g1 = h1 + l1/2*[sin(h1(3)); cos(h1(3)); 0];
f1 = h1 + l1*[sin(h1(3)); cos(h1(3)); 0];

% Right link (leg)
f3 = h2 + [0; 0; -a2];
g3 = f3 + l3/2*[-sin(f3(3)); -cos(f3(3)); 0];
h3 = f3 + l3*[-sin(f3(3)); -cos(f3(3)); 0];

% Center of Mass (CoM)
g = [(m1*g1(1:2) + m2*g2(1:2) + m3*g3(1:2))/(m1+m2+m3);th];

% g1 w.r.t. g
g2g = [solve((g(1)-xc),x); solve((g(2)-yc),y); g(3)];

%--- Velocities
dg2g = jacobian(g2g,q)*dq;
dg = jacobian(g,q2)*dq2;

dg2 = jacobian(g2,q2)*dq2;
df2 = jacobian(f2,q2)*dq2;
dh2 = jacobian(h2,q2)*dq2;

dg1 = jacobian(g1,q2)*dq2;
df1 = jacobian(f1,q2)*dq2;
dh1 = jacobian(h1,q2)*dq2;

dg3 = jacobian(g3,q2)*dq2;
df3 = jacobian(f3,q2)*dq2;
dh3 = jacobian(h3,q2)*dq2;

%--- Contact Positions
% ground point
v1 = [1;0]; v2 = [0;0];
plane = [v1,v2];

Phi1 = plane2pntDist(plane,f1);
Phi2 = plane2pntDist(plane,h1);
Phi3 = plane2pntDist(plane,f3);
Phi4 = plane2pntDist(plane,h3);

Phi = [Phi1;Phi2;Phi3;Phi4];

% Projection of constraint forces to generalized coordinates
J = jacobian(Phi,q2);

% Tangential contact velocities
Psi = J*dq2;
%% ---------------------------------------------------------
%   SYSTEM DYNAMICS
% ----------------------------------------------------------

%--- KINETIC ENERGY
KE1 = 0.5*m1*dg1.'*dg1 + 0.5*I1*dth.^2;
KE2 = 0.5*m2*dg2.'*dg2 + 0.5*I2*dth.^2;
KE3 = 0.5*m3*dg3.'*dg3 + 0.5*I3*dth.^2;

KE = KE1 + KE2 + KE3;
KE = simplify(KE);

%--- POTENTIAL ENERGY
PE = G*(m1*g1(2) + m2*g2(2) + m3*g3(2));
PE = simplify(PE);

%--- COMPUTING EOM MATRICIES
% Gravity vector
G_vec = simplify(jacobian(PE,q2).');

% Mass-Inertia matrix
D_mtx = simplify(jacobian(KE,dq2).');
D_mtx = simplify(jacobian(D_mtx,dq2));

% Coriolis and centrifugal matrix
syms C_mtx real
n=max(size(q2));
for k=1:n
    for j=1:n
        C_mtx(k,j) = 0*G;
        for i=1:n
            C_mtx(k,j) = C_mtx(k,j)+ 1/2*(diff(D_mtx(k,j),q2(i)) + ...
                diff(D_mtx(k,i),q2(j)) - ...
                diff(D_mtx(i,j),q2(k)))*dq2(i);
        end
    end
end
C_mtx = simplify(C_mtx);

% Input Matrix
Phi_0 = [a1;a2];
B_mtx = simplify(jacobian(Phi_0,q2));
B_mtx = B_mtx';

% ddq
%ddq2 = D_mtx\(-C_mtx*dq2 - G_vec + J'*lambda);

%% ----------------------------------------------------------
%   GENERATE MATLAB FUNCTIONS
% -----------------------------------------------------------
% Create folder to save autogerated functions
autoFolderName = 'autogenFncs';
if ~exist(autoFolderName,'dir')
  mkdir(autoFolderName);
end

params_list = {'m1','m2','m3',...
                'I1','I2','I3',...
               'l1','l2','l3',...
               'G'};
           
q_list1 = {'xc','yc','th','a1','a2'};
dq_list1 = {'dxc','dyc','dth','da1','da2'};

q_list2 = {'x','y','th','a1','a2'};
dq_list2 = {'dx','dy','dth','da1','da2'};

% Generate Kinematics
% --------------------

matlabFunction(D_mtx,C_mtx,G_vec,B_mtx,J,'File',fullfile(autoFolderName,'autogen_EOM.m'),...
                'vars',horzcat(q_list2,dq_list2,params_list));
            
matlabFunction(Phi,Psi,'File',fullfile(autoFolderName,'autogen_contactDyn.m'),...
                'vars',horzcat(q_list2,dq_list2,params_list));
            
matlabFunction(g2g,dg2g,'File',fullfile(autoFolderName,'autogen_g2g.m'),...
                'vars',horzcat(q_list1,dq_list1,params_list));
            
matlabFunction(g1,g2,g3,h1,h2,h3,f1,f2,f3,g,'File',fullfile(autoFolderName,'autogen_kinematics.m'),...
                'vars',horzcat(q_list2,params_list));
            
matlabFunction(dg1,dg2,dg3,dh1,dh2,dh3,df1,df2,df3,dg,'File',fullfile(autoFolderName,'autogen_velocities.m'),...
                'vars',horzcat(q_list2,dq_list2,params_list));
end

function d = plane2pntDist(plane, pnt)
    
    v1 = plane(:,1);
    v2 = plane(:,2);
    
    d =((v2(1)-v1(1))*(v1(2)-pnt(2)) - (v1(1)-pnt(1))*(v2(2)-v1(2)))/norm(v2-v1);
    d = simplify(d);
end
          

