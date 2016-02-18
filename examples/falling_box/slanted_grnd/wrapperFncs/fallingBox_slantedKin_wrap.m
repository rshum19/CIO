function [cntc_pts] = fallingBox_slantedKin_wrap(t,z,params)

p = params;

[Pc1,Pc2,Pc3] = autogen_contact_pts(z(1,:),z(2,:),z(3,:),...
                                    p.w,p.h,...
                                    p.m,p.I,...
                                    p.g);
cntc_pts = [Pc1; Pc2; Pc3];
end

