function [p0,pos] = FK_1(L1,L2,L3,q1,q2,q3,o1,theta)
    p0 = eye(4)*transl(o1(1),o1(2),o1(3))*trotz(theta);
    p1 = trotz(q1)*transl(L1,0,0);   
    p2 =troty(q2)*transl(L2,0,0);    
    p3 = troty(q3)*transl(L3,0,0);
    p1 = p0*p1;
    p21 = p1*p2;
    p31 = p21*p3;
    pos = [ p0(1:3,4) p1(1:3,4) p21(1:3,4) p31(1:3,4)];
end