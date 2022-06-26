clear all
close all
glvs;
%%
load( 'C:\04 MyCodes\MyCodes\data\20200906\静止用于allan\mat\footstatic.mat');
foot = footstatic;
foot_imu.acc = foot(:,1:3); %转为m/s^2
foot_imu.gyros = foot(:,4:6); %转为rad/s
foot_imu.ts = 1/100;

filename = 'C:\04 MyCodes\MyCodes\data\新主楼\BayesProj.xlsx';%'C:\04 MyCodes\MyCodes\data\DATA3.xlsx'
foot = xlsread(filename,1);

filename = 'C:\04 MyCodes\MyCodes\data\20220518\mystream_5_18_19_11_4.csv';
foot = xlsread(filename,1);

filename = 'C:\04 MyCodes\MyCodes\data\mxf_imu.csv';
foot = xlsread(filename,1);

filename = 'C:\04 MyCodes\MyCodes\data\aalto\IMU1r.csv';%'C:\04 MyCodes\MyCodes\data\DATA3.xlsx'
foot = xlsread(filename,1);

filename = 'C:\04 MyCodes\MyCodes\data\aalto\IMU5r.csv';%'C:\04 MyCodes\MyCodes\data\DATA3.xlsx'
foot = xlsread(filename,1);
foot_imu.ts = 0.05;

figure;hold on
plot(foot(:,1:3));
plot(foot(:,4:6));
plot(foot(:,11:13));

foot_imu.acc = (sSys2rfu('rfu')*foot(:,1:3)')' * glv.g0; %转为m/s^2
foot_imu.gyros = (sSys2rfu('rfu')*foot(:,4:6)')' * glv.rad; %转为rad/s
foot_imu.mag = (sSys2rfu('rfu')*foot(:,11:13)')'; %转为rad/s
foot_imu.ts = 1/100;
%%
filename = 'D:\MXFcodes\MATLAB\MARG-IMU-based-in-door-positioning\Human movement\Xiaofeng 1\VNYMR.csv';
foot = xlsread(filename,1);
foot_imu.acc = (sSys2rfu('frd')*foot(:,7:9)')'; %转为m/s^2
foot_imu.gyros = (sSys2rfu('frd')*foot(:,10:12)')'; %转为rad/s
foot_imu.mag = (sSys2rfu('frd')*foot(:,4:6)')'; %转为rad/s
foot_imu.ts = 1/50;


order = 'zxy';

foot_imu1 = foot_imu;
%---------------初始粗对准-----------------
still_time =100; 
[foot_q0,foot_att0] = alignbyAccMag(foot_imu1.acc(1:still_time,:),[],order);
disp(['-----  初始对准角度（°/s）：',num2str((foot_att0*glv.deg)'),'  -----']);
if abs(foot_att0(1))>0.1 || abs(foot_att0(2))>0.1 %0727
    foot_imu1.acc = (sSys2rfu(foot_att0,order)*foot_imu1.acc')'; %转为m/s^2
    foot_imu1.gyros = (sSys2rfu(foot_att0,order)*foot_imu1.gyros')'; %转为rad/s
%     foot_imu1.ratt = (sSys2rfu(foot_att0,order)*foot_imu1.ratt')'; %保持°
    [foot_q0,foot_att0] = alignbyAccMag(foot_imu1.acc(1:still_time,:),[],order); %根据严静止判断出的动前静止区间选取
    disp(['-----  初始对准角度（°/s）：',num2str((foot_att0*glv.deg)'),'  -----']);
end
foot_imu1.acc = foot_imu1.acc(still_time+1:end,:);%更新起点
foot_imu1.gyros = foot_imu1.gyros(still_time+1:end,:);

imu = foot_imu1;
qua0 = foot_q0;

BayesEKF

%%
N=length(imu.gyros);
t = 0:0.01:0.01*(N-1);
Ref = zeros(5,N) + [180;90;0;-90;-180];

xlabel('Time (min)');
ylabel('Angle (°)')
 set(gca,'FontSize',10,'FontName','Times New Roman');
 
 
figure
plot(t/60,att_EKF(:,1)*glv.deg,'-g','Linewidth',2);
hold on
plot(t/60,att_UKF(:,1)*glv.deg,'--c','Linewidth',1.5);
plot(t/60,att_GHKF(:,1)*glv.deg,'-.m','Linewidth',1);
plot(t/60,att_CKF(:,1)*glv.deg,':b','Linewidth',1);

figure
plot(t/60,att_EKF(:,2)*glv.deg,'-g','Linewidth',2);
hold on
plot(t/60,att_UKF(:,2)*glv.deg,'--c','Linewidth',1.5);
plot(t/60,att_GHKF(:,2)*glv.deg,'-.m','Linewidth',1);
plot(t/60,att_CKF(:,2)*glv.deg,':b','Linewidth',1);

figure
plot(t/60,att_EKF(:,3)*glv.deg,'-g','Linewidth',2);
hold on
plot(t/60,att_UKF(:,3)*glv.deg,'--c','Linewidth',1.5);
plot(t/60,att_GHKF(:,3)*glv.deg,'-.m','Linewidth',1);
plot(t/60,att_CKF(:,3)*glv.deg,':b','Linewidth',1);
plot(t/60,Ref,'r:','Linewidth',1.5,'color',[0.9,0.3,0.3])

legend({'EKF','UKF','GHKF','CKF','Reference'});

figure;hold on;box on;
plot(pos_corner(:,1),pos_corner(:,2),'-r','linewidth',1.5,'color',[0.9,0.3,0.3]);
plot(pose(:,1),pose(:,2),'-g','Linewidth',2);
plot(posu(:,1),posu(:,2),'--c','Linewidth',1.5);
plot(posgh(:,1),posgh(:,2),'-.m','Linewidth',1);
plot(posc(:,1),posc(:,2),':b','Linewidth',1);

legend({'Reference','EKF','UKF','GHKF','CKF'});
xlabel('X (m)');
ylabel('Y (m)')
 set(gca,'FontSize',10,'FontName','Times New Roman');