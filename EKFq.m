% ######################################
% ### orientation tracking using EKF ###
% ######################################
% by Xiaofeng Ma @ Aalto University
%%
global glv;
N=length(imu.gyros);

%% zero velociy detection
% [zvd,~] = zeroVelocityDetector(imu,config);
[gait_time,~,~,zvd,~,~] = gait_devide_ZVD_MXFforAtt(imu,glv.g0);

%% initial
% ### done in the main test file.

%% Filter   
MM = zeros(size(m,1),N);
PP = zeros(size(P,1),size(P,2),N); 
 
for k=1:N
    
    % ### prediction via dynamics
    [f,F] = f_qua(X_type,m,[imu.gyros(k,:),imu.ts],'Eular');

    m = f;
    P = F*P*F' + Q;
    m(1:4) = m(1:4)/norm(m(1:4));
    
    % ### measuremnet update
    if zvd(k)==1 
        [z_pre,z_obs,H] = h(X_type,m,obs_type,obs(k,:)',para);

        S = H*P*H' + R;
        K = P*H'/S;
        m = m+ K*(z_obs - z_pre);
        P = P - K*S*K';
        m(1:4) = m(1:4)/norm(m(1:4));
    end       
    P = (P + P')/2;    
    
    MM(:,k) = m;
    PP(:,:,k) = P;
end

t = 0:imu.ts:imu.ts*(N-1);
Ref = zeros(5,N) + [180;90;0;-90;-180];
    
figure
plot(t,MM,'Linewidth',1);
title('EKF estimate');

att_EKF = qua2attV(MM_e', 'zxy');
figure
plot(t/60,att_EKF*glv.deg,'Linewidth',1);
hold on;
plot(t/60,Ref,'r:','Linewidth',1.5)
legend('pitch','roll','yaw','reference');
title('EKF attitude estimates');

q = MM';

