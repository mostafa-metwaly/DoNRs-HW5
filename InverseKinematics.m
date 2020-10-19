function q=InverseKinematics(P_in)
 
L1=1;
L2=1;
L3=1;
px = P_in(:,1);
py = P_in(:,2);
pz = P_in(:,3);

 m= 1;%+1 for elbow up or -1 for elbow down;
for i = 1:length(px)

b =  L1-pz(i) ; 
a = sqrt(py(i)^2+px(i)^2);
q1(:,i) = atan2(py(i),px(i));
q2(:,i) = atan2(b,a) - m * acos((a^2 + b^2 + L2^2 - L3^2)/(2*L2*sqrt(a^2 + b^2)));
q3(:,i) = acos( ( a^2 + b^2 - L2^2 - L3^2)/ (2 * L2 * L3) );

end
 q=[q1; q2 ;q3]';
 
 