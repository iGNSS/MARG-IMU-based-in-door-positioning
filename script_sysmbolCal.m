syms q0 q1 q2 q3
syms theta gamma varphi %pitch roll yaw
syms I J K
syms vx vy vz
syms px py pz
syms Ts g
syms fbx fby fbz
syms wx wy wz
%% z - sky; x - right; y - forward
Cz = [cos(varphi) sin(varphi) 0;-sin(varphi) cos(varphi) 0;0 0 1];
Cx = [1 0 0;0 cos(theta) sin(theta);0 -sin(theta) cos(theta)];
Cy = [cos(gamma) 0 -sin(gamma);0 1 0;sin(gamma) 0 cos(gamma)];
%% zyx 顺序
Cbn = Cx*Cy*Cz
qa = cos(yaw/2)+sin(yaw/2)*K
qb = cos(roll/2)+sin(roll/2)*J
qc = cos(pitch/2)+sin(pitch/2)*I
qbn = qa*qb*qc
%% zxy 顺序
Cbn = Cy*Cx*Cz
qa = cos(yaw/2)+sin(yaw/2)*K
qb = cos(pitch/2)+sin(pitch/2)*I
qc = cos(roll/2)+sin(roll/2)*J
qbn = qa*qb*qc
expand(qbn)
%% xyz 顺序
Cbn = Cz*Cy*Cx
qa = cos(yaw/2)+sin(yaw/2)*K
qb = cos(pitch/2)+sin(pitch/2)*I
qc = cos(roll/2)+sin(roll/2)*J
qbn = qa*qb*qc
expand(qbn)
%% xzy 顺序
Cbn = Cy*Cz*Cx
%% f
Cnb = qua2dcm([q0 q1 q2 q3],'Cnb')
Cbn = qua2dcm([q0 q1 q2 q3],'Cbn')
fp = Ts*[vx;vy;vz]+[px;py;pz];
fv = Ts*(Cnb*[fbx;fby;fbz]-[0;0;g])+[vx;vy;vz];
fq = fQuat([q0 q1 q2 q3],[wx wy wz Ts],0);
Jfq = jacobian(fq,[q0 q1 q2 q3])
f = [fq;fp;fv]
Jf = jacobian(f,[q0 q1 q2 q3 px py pz vx vy vz])

%% h
hpv = [px;py;pz;vx;vy;vz]
Jhpv = jacobian(hpv,[q0 q1 q2 q3 px py pz vx vy vz])

hgn = Cbn*[0;0;g]
Jhgn = jacobian(hgn,[q0 q1 q2 q3 px py pz vx vy vz])


hvx0 = Cbn*[vx;vy;vz]
hvx = [hvx0(1);hvx0(3)]

h = [hgn;hvx]
Jh = jacobian(h,[q0 q1 q2 q3 px py pz vx vy vz])