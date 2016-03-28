function [ c_comp, ceq_comp] = bndCst(z,params)

% Read Input
z0 = z(:,1);
% Forward Kinematics
[p1,p2,p3,com] = kin_wrt_link2_wrap(z0,params);

%% INEQUALITY CONSTRAINTS

%% EQUALITY CONSTRAINTS

%--- S.T. foot is always on ground
ceq_foot = p3.h(2);


% Concatenate constraints
c_comp = [];
ceq_comp = ceq_foot;

end
