function [EoM] = fallingBox_slantedDyn_wrap2(t,z,lambda,params)


%% ----------------------------------------------------------
%   READ AND INTERPRET INPUTS
% -----------------------------------------------------------
nt = length(t);
p = params;
nc = size(lambda,1);

%% ----------------------------------------------------------
%   EQUATION OF MOTION
% -----------------------------------------------------------
% Allocate memory
Phi = zeros(nc,nt);
Psi = zeros(nc,nt);
J = zeros(size(z,1)/2,nt);
D_mtx = cell(1,nt);
C_mtx = cell(1,nt);
G_vec = cell(1,nt);
B_mtx = cell(1,nt);
for i = 1:nt
 
    % Contact dynamic matrices Phi, Psi, J
    [Phi(:,i),Psi(:,i),J(:,i)] = fallingBox_contactDyn_wrapper(z(:,i),p);
    
    % EOM
    [D_mtx{i},C_mtx{i},G_vec{i},B_mtx{i}] = autogen_fallingBox_EOM_mtxs(z(1,i),z(2,i),z(3,i),...
                                                              z(4,i),z(5,i),z(6,i),...
                                                              p.w,p.h,...
                                                              p.m,p.I,...
                                                              p.g);
end

EoM.D_mtx = D_mtx;
EoM.C_mtx = C_mtx;
EoM.G_vec = G_vec;
EoM.B_mtx = B_mtx;
EoM.Phi = Phi;
EoM.Psi = Psi;
EoM.J = J;

end

