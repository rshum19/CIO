function [F, Phi, Psi] = fallingBox_slantedDyn_wrap(t,z,u,lambda,params)


%% ----------------------------------------------------------
%   READ AND INTERPRET INPUTS
% -----------------------------------------------------------
nt = length(t);
p = params;
x = z(1:3,:);
dx = z(4:6,:);
g = p.g;

% Mass matrix
M = diag([p.m,p.m,p.I]);

% Gravity vector
G = [0;p.m*g;0];  

if isempty(lambda)
    lambda = zeros(1,nt);
end
nc = size(lambda,1);
%% ----------------------------------------------------------
%   EQUATION OF MOTION
% -----------------------------------------------------------
% Allocate memory
Phi = zeros(nc,nt);
Psi = zeros(nc,nt);
ddx = zeros(size(x,1),nt);
ddx2 = zeros(size(x,1),nt);
ddx3 = zeros(size(x,1),nt);
J = zeros(size(x,1),nt);
for i = 1:nt
 
    % Contact dynamic matrices Phi, Psi, J
    [Phi(:,i),Psi(:,i),J(:,i)] = fallingBox_contactDyn_wrapper(z(:,i),p);
    
    % EOM
    [D_mtx,C_mtx, G_vec] = autogen_fallingBox_EOM_mtxs(z(1,i),z(2,i),z(3,i),...
                                                              z(4,i),z(5,i),z(6,i),...
                                                              p.w,p.h,...
                                                              p.m,p.I,...
                                                              p.g);
    
    %ddx2(:,i) = D_mtx\(- G_vec + J(:,i)'*lambda(:,i));       
    ddx(:,i) = autogen_fallingBox_ddq(z(1,i),z(2,i),z(3,i),...
                                       z(4,i),z(5,i),z(6,i),...
                                       lambda(:,i),...
                                       p.w,p.h,...
                                       p.m,p.I,...
                                       p.g,...
                                       p.line(1),p.line(2));
end

F = ddx;
end

