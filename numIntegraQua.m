
X0 = qua0;
X = zeros(size(X0,1),N);    
for k=1:N
    
    X(:,k) = f_qua('q',X0,[imu.gyros(k,:),imu.ts],'exp');
    X0 = X(1:4,k)/norm(X(1:4,k));
    X(1:4,k) = X0;
end

X = X';

figure
plot(t,X(:,1:4),'Linewidth',1);
title('EKF estimate');

att_num = qua2attV(X(:,1:4), 'zxy');
figure
plot(t/60,att_num*glv.deg,'Linewidth',1);
hold on;
plot(t/60,Ref,'r:','Linewidth',1.5)
legend('pitch','roll','yaw','reference');
title('Numerical attitude estimates');

q = X(:,1:4);