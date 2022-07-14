clear all
close all
glvs;
%%
load( 'C:\04 MyCodes\MyCodes\data\20200906\静止用于allan\mat\footstatic.mat');
foot = footstatic;
foot_imu.acc = foot(:,1:3); %转为m/s^2
foot_imu.gyros = foot(:,4:6); %转为rad/s
foot_imu.ts = 1/100;

filename = 'D:\04 MyCodes\MyCodes\data\新主楼\BayesProj.xlsx';
foot = xlsread(filename,1);

filename = 'C:\04 MyCodes\MyCodes\data\20220518\mystream_5_18_19_11_4.csv';
foot = xlsread(filename,1);

filename = 'C:\04 MyCodes\MyCodes\data\mxf_imu.csv';
foot = xlsread(filename,1);

filename = 'C:\04 MyCodes\MyCodes\data\aalto\IMU1r.csv';
foot = xlsread(filename,1);

filename = 'C:\04 MyCodes\MyCodes\data\aalto\IMU5r.csv';
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
figure(1002);hold on;
for trial=1:10
filename = ['D:\MXFcodes\MATLAB\MARG-IMU-based-in-door-positioning\Human movement\Xiaofeng ',num2str(trial),'\VNYMR.csv'];
% filename = ['D:\MXFcodes\MATLAB\MARG-IMU-based-in-door-positioning\calibration\static ',num2str(trial),'\VNYMR.csv'];
foot = xlsread(filename,1);
foot_imu.acc = (sSys2rfu('frd')*foot(:,7:9)')'; %转为m/s^2
foot_imu.gyros = (sSys2rfu('frd')*foot(:,10:12)')'; %转为rad/s
foot_imu.mag = (sSys2rfu('frd')*foot(:,4:6)')'; %转为rad/s
foot_imu.ts = 1/50;

%--------------calibration---------------
% mag
% foot_imu.mag = (foot_imu.mag-b)*A; 
% VN100 acc
scale_acc = [1.0007    0.0186   -0.0031;
   -0.0058    1.0009   -0.0005;    
    0.0179   -0.0015    1.0010];    
bias0_acc = [-0.0498;0.0129;0.4891];
foot_imu.acc = (inv(scale_acc)*(foot_imu.acc'-bias0_acc))'; 

% gyro
still_time =50; 
bias0_gyro = mean(foot_imu.gyros(1:still_time,:));

%-----------------------------
order = 'zxy';

foot_imu1 = foot_imu;
%---------------initial alignment-----------------

[foot_q0,foot_att0] = alignbyAccMagatStatic(foot_imu1.acc(1:still_time,:),[],order);
disp(['-----  initial attitude（deg/s）：',num2str((foot_att0*glv.deg)'),'  -----']);
if abs(foot_att0(1))>0.1 || abs(foot_att0(2))>0.1 %0727
    foot_imu1.acc = (sSys2rfu(foot_att0,order)*foot_imu1.acc')'; %转为m/s^2
    foot_imu1.gyros = (sSys2rfu(foot_att0,order)*foot_imu1.gyros')'; %转为rad/s
%     foot_imu1.ratt = (sSys2rfu(foot_att0,order)*foot_imu1.ratt')'; %保持°
    [foot_q0,foot_att0] = alignbyAccMagatStatic(foot_imu1.acc(1:still_time,:),[],order); %根据严静止判断出的动前静止区间选取
    disp(['-----  initial attitude（deg/s）：',num2str((foot_att0*glv.deg)'),'  -----']);
end

foot_imu1.acc = foot_imu1.acc(still_time+1:end,:);%更新起点
foot_imu1.gyros = foot_imu1.gyros(still_time+1:end,:);

imu = foot_imu1;
qua0 = foot_q0;

X_type = 'q bG'; 
obs_type = 'gn'; obs = imu.acc; para = [0;0;glv.g0];
[m,P,Q,R] = filterInitial(X_type,[qua0;bias0_gyro'],[],[],obs_type,[]);

EKFq
BayesEKF
numIntegraQua

N = length(q);
accn = zeros(N,3);
    for i = 1:N
         accn(i,:) = (qua2dcm(q(i,:),'Cnb')*imu.acc(i,:)' - [0;0;glv.g0])';
    end
    
 pos = posCalculateWithGait(imu,accn');
 
 figure(1002);hold on;
 plot(pos(:,1),pos(:,2),'b-')
end 
% [pose,~,~,~,~,arr_gait_time,stationaryStart,stationaryEnd,stationary,~]=positionCalculate(imu,accne','foot');
% figure;plot(pose(:,1),pose(:,2))
% [pose]=posCalculate(imu,MM_e');
% figure;plot(pose(:,1),pose(:,2))

figure
plot3(pose(:,1),pose(:,2),pose(:,3))

%% yaw calculated by magnetometer
yaw = zeros(N,1);
for k=1:N
    [foot_q0,foot_att0] = alignbyAccMagatStatic(foot_imu1.acc(k,:),foot_imu1.mag(k,:),order);
    yaw(k,1) = foot_att0(3);
end
figure
plot(1:N,yaw)

att_EKF(:,3) = yaw;
accne = zeros(N,3);
for i = 5:N
    accne(i,:) = (att2dcm(MM_e(:,i)',order)*imu.acc(i,:)' - [0;0;glv.g0])';
end
    
 pose = posCalculateWithGait(imu,accne');
 figure(1000)
 plot(pose(:,1),pose(:,2),'g-')

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
box on
xlabel('X (m)');
ylabel('Y (m)')
 set(gca,'FontSize',10,'FontName','Times New Roman');
 ref_pos = [0,0;0,1.2*8;-1.2*2,1.2*8;-1.2*2,0;0,0];
 plot(ref_pos(:,1),ref_pos(:,2),'g')
 

