function q=InverseKinematics(P_in)


L1=1;
L2=1;
L3=1;
px = P_in(:,1);
py = P_in(:,2);
pz = P_in(:,3);

 m= 1;%+1 for elbow up or -1 for elbow down;
% px = cos(q1)* (L2*cos(q2)+ L3*cos(q2+q3));
% py = sin(q1)* (L2*cos(q2)+ L3*cos(q2+q3));
% pz = L1+ L2*sin(q1)+ L3*sin(q2+q3);


for i = 1:length(px)

b = pz(i) - L1; 
a = sqrt(py(i)^2+px(i)^2);
q1  = atan2(py(i),px(i));
if q1 >= pi
    q1(:,i) = q1-pi;
else 
    q1(:,i) = atan2(py(i),px(i));

end

q2(:,i) = atan2(b,a) - m * acos((a^2 + b^2 + L2^2 - L3^2)/(2*L2*sqrt(a^2 + b^2)));
q3(:,i) = acos( ( a^2 + b^2 - L2^2 - L3^2)/ (2 * L2 * L3) );


 

end
 

 q=[q1; q2 ;q3]';
% %2 solutions for q1:
% %when px^2+py^2 is greater than zero
% 
% q11=atan2(py,px)
% q12=atan2(-py,-px)
% 
%  
% 
% %2 solutions for q3:
% 
% q31=atan2(sin(q3),cos(q3));
% q32=atan2(-sin(q3),cos(q3));
 