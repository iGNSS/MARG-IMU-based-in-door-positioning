syms q0 q1 q2 q3 % quaternion
syms theta gamma varphi %pitch roll yaw
syms I J K % unit vector
syms vx vy vz % velocity
syms px py pz % position
syms Ts g % sampling time and gavity constant
syms fbx fby fbz % specific force (output of accelerometers)
syms wx wy wz % angular velocity (output of gyroscopes)
syms mx my mz % magnetic field (output of magnetometers)
syms bGx bGy bGz
syms bAx bAy bAz
%% z - sky; x - right; y - forward
Cz = [cos(varphi) sin(varphi) 0;-sin(varphi) cos(varphi) 0;0 0 1];
Cx = [1 0 0;0 cos(theta) sin(theta);0 -sin(theta) cos(theta)];
Cy = [cos(gamma) 0 -sin(gamma);0 1 0;sin(gamma) 0 cos(gamma)];
%% zyx order
Cbn = Cx*Cy*Cz
qa = cos(yaw/2)+sin(yaw/2)*K
qb = cos(roll/2)+sin(roll/2)*J
qc = cos(pitch/2)+sin(pitch/2)*I
qbn = qa*qb*qc
%% zxy order
Cbn = Cy*Cx*Cz
qa = cos(yaw/2)+sin(yaw/2)*K
qb = cos(pitch/2)+sin(pitch/2)*I
qc = cos(roll/2)+sin(roll/2)*J
qbn = qa*qb*qc
expand(qbn)
%% xyz order
Cbn = Cz*Cy*Cx
qa = cos(yaw/2)+sin(yaw/2)*K
qb = cos(pitch/2)+sin(pitch/2)*I
qc = cos(roll/2)+sin(roll/2)*J
qbn = qa*qb*qc
expand(qbn)
%% xzy order
Cbn = Cy*Cz*Cx
%% f & F
Cnb = qua2dcm([q0 q1 q2 q3],'Cnb')
Cbn = qua2dcm([q0 q1 q2 q3],'Cbn')

fp = Ts*[vx;vy;vz]+[px;py;pz];

fv = Ts*(Cnb*[fbx;fby;fbz]-[0;0;g])+[vx;vy;vz];

fq = f_qua([q0;q1;q2;q3],[wx wy wz Ts],'Eular','q');
Fq = jacobian(fq,[q0 q1 q2 q3])

fqbw = f_qua([q0;q1;q2;q3;bGx;bGy;bGz],[wx wy wz Ts],'Eular','q bG');
Fqbw = jacobian(fqbw,[q0 q1 q2 q3 bGx bGy bGz])

fqbwa = f_qua([q0;q1;q2;q3;bGx;bGy;bGz;bAx;bAy;bAz],[wx wy wz Ts],'Eular','q bG bA');
Fqbwa = jacobian(fqbwa,[q0 q1 q2 q3 bGx bGy bGz bAx bAy bAz])

f = [fq;fp;fv]
Fqpv = jacobian(f,[q0 q1 q2 q3 px py pz vx vy vz])

%% h & H
hpv = [px;py;pz;vx;vy;vz]
Hpv = jacobian(hpv,[q0 q1 q2 q3 px py pz vx vy vz])

hgn = Cbn*[0;0;g]
Hgn = jacobian(hgn,[q0 q1 q2 q3 px py pz vx vy vz])


hvx0 = Cbn*[vx;vy;vz]
Hvx = [hvx0(1);hvx0(3)]

h = [hgn;hvx]
H = jacobian(h,[q0 q1 q2 q3 px py pz vx vy vz])

hn = Cnb*[mx;my;mz]
hnx = mx*(q0^2+q1^2-q2^2-q3^2)+2*my*(q1*q2-q0*q3)+2*mz*(q1*q3+q0*q2)
Jhnx = jacobian(hnx,[q0 q1 q2 q3])

hq = h('q',[q0;q1;q2;q3],'gn',[fbx;fby;fbz],['0';'0';'g']);
Hq = jacobian(hq,[q0 q1 q2 q3])
% Hq = vpa(Hq,6)

hqbw = h('q bG',[q0;q1;q2;q3;bGx;bGy;bGz],'gn',[fbx;fby;fbz],['0';'0';'g']);
Hqbw = jacobian(hqbw,[q0 q1 q2 q3 bGx bGy bGz])

hqbwa = h('q bG bA',[q0;q1;q2;q3;bGx;bGy;bGz;bAx;bAy;bAz],'gn',[fbx;fby;fbz],['0';'0';'g']);
Hqbwa = jacobian(hqbwa,[q0 q1 q2 q3 bGx bGy bGz bAx bAy bAz])
