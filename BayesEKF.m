global glv;

%%
% Simulate data
%
N=length(imu.gyros);
mg=glv.g0*0.001;

%-------璁剧疆鍒濆PQR鍜岀姸鎬佸垵鍊?-----
  %09.19  
ARW=[0.356269993661301;0.399565098420031;0.322625636319760];%deg/sqrt(h)
VRW = [0.0678972113063788;0.0684746098011766;0.0846659351618287];    
BIg = [31.5227989474692;22.3369625303739;30.6826396871611];
BIa = [3.27530955305814;6.61103955408311;15.4628670415549];
SigmaG = diag((ARW/60*pi/180).^2);
SigmaA=diag((VRW/60).^2); %
BIg = BIg*pi/180/3600;
BIa = BIa/3600;
dph=4.8481e-6;dphpsh=8.0802e-8;

%% zero velociy detection
%判断零速率状态
% [zvd,~] = zeroVelocityDetector(imu,config);
[~,~,~,zvd,~,~] = gait_devide_ZVD_MXFforAtt(imu,glv.g0);

%%
% Filter
%
m = qua0;
P = diag([0.01;0.01;0.01;0.01]);
Q = diag([0;0;0;0]);%([0.5;0.5;0.5;0.5]);
J1 = qua2dcm(qua0,'Cnb');
R=J1*blkdiag(SigmaA)*J1'*1000;%以fb为量测时1000
% R = 0.002;

MM = zeros(size(m,1),N);
PP = zeros(size(P,1),size(P,2),N); 
    
for k=1:N
    f = fx4(m, [imu.gyros(k,:) imu.ts]);
    F = eye(4)+0.5*imu.ts.*[0 -imu.gyros(k,1) -imu.gyros(k,2)  -imu.gyros(k,3);  
                            imu.gyros(k,1) 0  imu.gyros(k,3) -imu.gyros(k,2);
                            imu.gyros(k,2)  -imu.gyros(k,3) 0  imu.gyros(k,1);
                            imu.gyros(k,3)  imu.gyros(k,2) -imu.gyros(k,1) 0];

    m = f;
    P = F*P*F' + Q;
    m = m/norm(m);
  
    % The eastward component of the geomagnetic field is zero
%     mx = imu.mag(k,1); my = imu.mag(k,2); mz = imu.mag(k,3);
%     q0 = m(1); q1 = m(2); q2 = m(3); q3 = m(4);
%     hm = imu.mag(k,1)*(m(1)^2+m(2)^2-m(3)^2-m(4)^2)+...
%         2*imu.mag(k,2)*(m(2)*m(3)-m(1)*m(4))+...
%         2*imu.mag(k,3)*(m(2)*m(4)+m(1)*m(3));
%     Hm = [ 2*mx*q0 - 2*my*q3 + 2*mz*q2, 2*mx*q1 + 2*my*q2 + 2*mz*q3, 2*my*q1 - 2*mx*q2 + 2*mz*q0, 2*mz*q1 - 2*my*q0 - 2*mx*q3];
%     S = Hm*P*Hm' + R;
%     K = P*Hm'/S;
%     m = m+ 0.2*K*(0 - hm);
%     P = P - K*S*K';
%     m = m/norm(m);
    
    % specific force = gravity in ZVI
    if zvd(k)==1 
        z = imu.acc(k,:)';
        h = hx4(m, glv.g0);
        H = 2*glv.g0*[-m(3) m(4) -m(1) m(2);      % Observation matrix
                      m(2) m(1) m(4) m(3);
                      m(1) -m(2) -m(3) m(4)];

        S = H*P*H' + R;
        K = P*H'/S;
        m = m+ 0.002*K*(z - h);
        P = P - K*S*K';
        m = m/norm(m);
    end       
    P = (P + P')/2;    
    
    MM(:,k) = m;
    PP(:,:,k) = P;
end

MM_e = MM;
t = 0:0.01:0.01*(N-1);
Ref = zeros(5,N) + [180;90;0;-90;-180];
    
figure
plot(t,MM_e,'Linewidth',1);
title('EKF estimate');

att_EKF = qua2attV(MM_e', 'zxy');
figure
plot(t/60,att_EKF*glv.deg,'Linewidth',1);
hold on;
plot(t/60,Ref,'r:','Linewidth',1.5)
legend('pitch','roll','yaw','reference');
title('EKF attitude estimates');


if 0
%% smoother
    ms = m;
    Ps = P;
    MMS = zeros(size(m,1),length(Y));
    PPS = zeros(size(P,1),size(P,2),length(Y));
    MMS(:,end) = m;
    PPS(:,:,end) = P;
    for k=size(MM,2)-1:-1:1
        m = MM(:,k);
        P = PP(:,:,k);
        f = fx4(m, imu.gyros(k,:));
        F = eye(4)+0.5*imu.ts.*[0 -imu.gyros(k,1) -imu.gyros(k,2)  -imu.gyros(k,3);  
                        imu.gyros(k,1) 0  imu.gyros(k,3) -imu.gyros(k,2);
                        imu.gyros(k,2)  -imu.gyros(k,3) 0  imu.gyros(k,1);
                        imu.gyros(k,3)  imu.gyros(k,2) -imu.gyros(k,1) 0];
    
        mp = f;
        mp = mp/norm(mp);
        Pp = F*P*F'+Q;
        Ck = P*F'/Pp;
        ms = m + Ck*(ms - mp);
        ms = ms/norm(ms);
        Ps = P + Ck*(Ps - Pp)*Ck';
        MMS(:,k) = ms;
        PPS(:,:,k) = Ps;
        Ps = (Ps + Ps')/2;
    end

att_EKS = qua2attV(MMS', 'zxy');
figure
plot(t/60,att_EKS*glv.deg,'Linewidth',1);
hold on;
plot(t/60,Ref,'r:','Linewidth',1.5)
legend('pitch','roll','yaw','reference');
title('EKF attitude estimates');

N = length(MMS');
accneS = zeros(N,3);
att = zeros(N,3);
    for i = 3:N
        att(i,:) = qua2att(MMS(:,i)','zxy');%0727
        accneS(i,:) = (qua2dcm(MMS(:,i)')*imu.acc(i,:)' - [0;0;glv.g0])';
    end
[poses,~,~,~,~,arr_gait_time,stationaryStart,stationaryEnd,stationary,~]=positionCalculate(imu,accneS','foot');
figure;plot(poses(:,1),poses(:,2))
    
    h = plot(T,Y,'k.',T,X(1,:),'r-',...
             T,MM(1,:),'b--',T,MMS(1,:),'g--');
    set(h,'Linewidth',5);
    title('EKF and ERTS estimates');
    legend('Measurements','True','EKF','ERTS');
    
    rmse_erts = sqrt(mean((X(1,:)-MMS(1,:)).^2))
end