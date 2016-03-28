function [link1,link2,link3,com] = kin_wrt_CoM_wrap(z,dz,params )
% Outputs:
%   link1:
%   link2:
%   link3:
%   g:      CoM position
% Inputs:
%   z:  [x,y,th,a1,a2]' generalized system coordinates
%   (x,y): center of mass catersian co-ordinate
%   th: link 2 orientation 
%   a1: relative angle between link 1 and 2
%   a2: relative angle between link 2 and 3
%

% Read inputs
p = params;

% Link 2 position w.r.t. CoM
[g2g,dg2g] = autogen_g2g(z(1),z(2),z(3),z(4),z(5),...
                         dz(1),dz(2),dz(3),dz(4),dz(5),...
                         p.m1,p.m2,p.m3,...
                         p.l1,p.l2,p.l3);

% Positions w.r.t to Link 2
[g1,g2,g3,h1,h2,h3,f1,f2,f3,g] = autogen_kinematics(g2g(1),g2g(2),g2g(3),z(4),z(5),...
                                                    p.m1,p.m2,p.m3,...
                                                    p.l1,p.l2,p.l3);
% Velocities w.r.t to Link 2                                                
[dg1,dg2,dg3,dh1,dh2,dh3,df1,df2,df3,dg] = autogen_velocities(g2g(1),g2g(2),g2g(3),z(4),z(5),...
                                                              dg2g(1),dg2g(2),dg2g(3),dz(4),dz(5),...
                                                              p.m1,p.m2,p.m3,...
                                                              p.l1,p.l2,p.l3);                                              
link1 = constructLink (g1,h1,f1,dg1,dh1,df1);
link2 = constructLink (g2,h2,f2,dg2,dh2,df2);  
link3 = constructLink (g3,h3,f3,dg3,dh3,df3); 

com.g = g;
com.dg = dg;
end

function link = constructLink (g,h,f,dg,dh,df)

% Positions
link.g = g;
link.h = h;
link.f = f;

% Velocities
link.dg = dg;
link.dh = dh;
link.df = df;

end
