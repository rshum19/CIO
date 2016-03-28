function [link1,link2,link3,com] = kin_wrt_link2_wrap(z,params)
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
nStates = size(z,1)/2;
q = z(1:nStates,:);
dq = z(nStates+1:end,:);

% Positions w.r.t to Link 2
[g1,g2,g3,h1,h2,h3,f1,f2,f3,g] = autogen_kinematics(q(1),q(2),q(3),q(4),q(5),...
                                                    p.m1,p.m2,p.m3,...
                                                    p.I1,p.I2,p.I3,...
                                                    p.l1,p.l2,p.l3,...
                                                    p.g);
                                                
% Velocities w.r.t to Link 2                                                
[dg1,dg2,dg3,dh1,dh2,dh3,df1,df2,df3,dg] = autogen_velocities(q(1),q(2),q(3),q(4),q(5),...
                                                              dq(1),dq(2),dq(3),dq(4),dq(5),...
                                                              p.m1,p.m2,p.m3,...
                                                              p.I1,p.I2,p.I3,...
                                                              p.l1,p.l2,p.l3,...
                                                              p.g);
                                                                                                                  
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


