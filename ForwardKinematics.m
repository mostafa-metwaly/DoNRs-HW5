function FK=ForwardKinematics(q_in)


 
L1=1;
L2=1;
L3=1;
q1 = q_in(:,1);
q2 = q_in(:,2);
q3 = q_in(:,3);


for i = 1:length(q1)
H= (Rz(q1(i))*Tz(L1)*Ry(q2(i))*Tx(L2)*Ry(q3(i))*Tx(L3));
T=H(1:3,4);
px(:,i)=T(1);
py(:,i)=T(2);
pz(:,i)=T(3);
 
end
FK=[px;py;pz]';

% px = cos(q1)* (L2*cos(q2)+ L3*cos(q2+q3));
% py = sin(q1)* (L2*cos(q2)+ L3*cos(q2+q3));
% pz = L1+ L2*sin(q1)+ L3*sin(q2+q3);