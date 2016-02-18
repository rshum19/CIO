function [ Pc1,Pc2,Pc3 ] = fallingBox_contactPts_wrap(t,z,params)

nt = length(t);
p = params;
x = z(1,:);
y = z(2,:);
th = z(3,:);

[Pc1,Pc2,Pc3] = autogen_contact_pts(x,y,th,...
                                    p.w,p.h,...
                                    p.m,p.I,...
                                    p.g);

end

