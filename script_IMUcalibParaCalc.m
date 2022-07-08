%% accelerometer
% ### d/u means the positive axis is facing downwards/upwards vertically on the ground
filename = 'D:\MXFcodes\MATLAB\MARG-IMU-based-in-door-positioning\calibration\acc xpositive calib\VNYMR.csv';
data = xlsread(filename,1);acc_xd = data(1:1724,7:9);gyro_xd = data(1:1724,10:12);
filename = 'D:\MXFcodes\MATLAB\MARG-IMU-based-in-door-positioning\calibration\acc xnegetive calib\VNYMR.csv';
data = xlsread(filename,1);acc_xu = data(1:1724,7:9);gyro_xu = data(1:1724,10:12);
filename = 'D:\MXFcodes\MATLAB\MARG-IMU-based-in-door-positioning\calibration\acc ypositive calib\VNYMR.csv';
data = xlsread(filename,1);acc_yd = data(1:1724,7:9);gyro_yd = data(1:1724,10:12);
filename = 'D:\MXFcodes\MATLAB\MARG-IMU-based-in-door-positioning\calibration\acc ynegetive calib\VNYMR.csv';
data = xlsread(filename,1);acc_yu = data(1:1724,7:9);gyro_yu = data(1:1724,10:12);
filename = 'D:\MXFcodes\MATLAB\MARG-IMU-based-in-door-positioning\calibration\acc zpositive calib\VNYMR.csv';
data = xlsread(filename,1);acc_zd = data(1:1724,7:9);gyro_zd = data(1:1724,10:12);
filename = 'D:\MXFcodes\MATLAB\MARG-IMU-based-in-door-positioning\calibration\acc znegetive calib\VNYMR.csv';
data = xlsread(filename,1);acc_zu = data(1:1724,7:9);gyro_zu = data(1:1724,10:12);

figure
plot(acc_xu);hold on;plot(acc_xd);
plot(acc_yu);plot(acc_yd);
plot(acc_zu);plot(acc_zd);

% figure
% plot(gyro_xu);hold on;plot(gyro_xd);
% plot(gyro_yu);plot(gyro_yd);
% plot(gyro_zu);plot(gyro_zd);

% ### calculate the mean of each axis
axdm = mean(acc_xd);axum = mean(acc_xu);
aydm = mean(acc_yd);ayum = mean(acc_yu);
azdm = mean(acc_zd);azum = mean(acc_zu);

% wxpm = mean(gyro_xu);wxnm = mean(gyro_xd);
% wypm = mean(gyroyp);wynm = mean(gyroyn);
% wzpm = mean(gyrozp);wznm = mean(gyrozn);
% 
% figure;plot([1,2,3],[wxpm;wxnm;wypm;wynm;wzpm;wznm])

% ### Six Point Method 
% ### S = [K b], f_calib = K^(-1)*(f_raw-b); 
Ao = [axum' axdm' ayum' aydm' azum' azdm'];
Ai = [[[9.80665;0;0],[-9.80665;0;0],[0;9.80665;0],[0;-9.80665;0],[0;0;9.80665],[0;0;-9.80665]];[1 1 1 1 1 1]];
S = Ao*Ai'*inv(Ai*Ai');

% ### Two Point Method
% ??? calculated bias differs from the bias calculated by the six point method
kx = (axum(1)-axdm(1))/2/glv.g0;
ky = (ayum(2)-aydm(2))/2/glv.g0;
kz = (azum(3)-azdm(3))/2/glv.g0;
bx = (axum(1)+axdm(1))/2/glv.g0;
by = (ayum(2)+aydm(2))/2/glv.g0;
bz = (azum(3)+azdm(3))/2/glv.g0;

%% magnetometer
filename = ['D:\MXFcodes\MATLAB\MARG-IMU-based-in-door-positioning\calibration\mag_fit_out\VNYMR.csv'];
data = xlsread(filename,1);

mag = data(:,4:6);


[A,b,expmfs] = magcal(mag); % calibration coefficients
expmfs % Dipaly expected  magnetic field strength in uT

C = (mag-b)*A; % calibrated data

figure(1)
plot3(mag(:,1),mag(:,2),mag(:,3),'LineStyle','none','Marker','X','MarkerSize',8)
hold on
grid(gca,'on')
plot3(C(:,1),C(:,2),C(:,3),'LineStyle','none','Marker', ...
            'o','MarkerSize',8,'MarkerFaceColor','r') 
axis equal
xlabel('uT')
ylabel('uT')
zlabel('uT')
legend('Uncalibrated Samples', 'Calibrated Samples','Location', 'southoutside')
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
hold off
