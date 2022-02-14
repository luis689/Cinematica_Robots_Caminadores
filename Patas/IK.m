function [theta1,theta2,theta3] = IK(h,x0,y0,L1,L2,L3,x,y,z)


theta1 = atan2(y-y0,x-x0);

r1 = sqrt((x-x0)^2+(y-y0)^2);
r2 = r1-L1;
r3 = h-z;
r4 = sqrt(r2^2+r3^2);
D = (r4^2-L2^2-L3^2)/(2*L2*L3);
if 1-D^2<0
    theta2 = -1000;
    theta3 = -1000;
else
    theta3 = atan2(sqrt(1-D^2),D);
    
    alpha = atan2(r3,r2);
    beta = atan2(L3*sin(theta3),L2+L3*cos(theta3));
    %beta = acosd((r4^2+L2^2-L3)/(2*r4*L2));
    theta2 = (alpha-beta);
end
end