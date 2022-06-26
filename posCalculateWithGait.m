function  [pos,V_correct,arr_gait_time,stationaryStart,stationaryEnd,stationary,Test_statistics]=posCalculateWithGait(imu,a)
%输入imu: 三轴加速度、三轴角速度、三轴磁强大小
%输入a:剔除重力后的三轴加速度
%输出position_x,position_y,position_z：位置的三维坐标
%输出arr_gait_time：站立期开始和结束采样点组成的矩阵
%输出stationaryStart,stationaryEnd：站立期结束和开始的采样点
%输出stationary：检测出的摆动期采样点和站立期采样点
%输出Test_statistics：加速度和角速度共同计算的统计量

%步态周期划分
global glv;
g=glv.g0;
% imu = foot_imu1;a = foot_accn';
[arr_gait_time,stationaryStart,stationaryEnd,stationary,Test_statistics,~]=gait_devide_ZVD_MXF(imu,g);
% figure;plot(foot_imu1.gyros(:,1));hold on;plot(stationary);
N=length(a);
n=length(arr_gait_time);
t = 0:imu.ts:imu.ts*(length(imu.acc)-1);
%%
%速度计算
Vx = zeros(1,N);
Vy = zeros(1,N);
Vz = zeros(1,N);
V = zeros(3,N);
for i=1:1:N-1
V(:,i+1)= V(:,i)+(a(:,i)+a(:,i+1))*imu.ts/2;   
end
figure;
subplot(311);plot(t',V(1,:)',':','linewidth',1.5,'color',RGB('#333333'));hold on;
subplot(312);plot(t',V(2,:)',':','linewidth',1.5,'color',RGB('#333333'));hold on;
subplot(313);plot(t',V(3,:)',':','linewidth',1.5,'color',RGB('#333333'));hold on;
suptitle('对所有数据积分得到的速度')
for i=1:1:n
    for j=arr_gait_time(1,i):arr_gait_time(2,i)%对摆动期进行积分
     Vx(1,j+1) =Vx(1,j)+ (a(1,j) + a(1,j+1))*imu.ts/2;
     Vy(1,j+1) =Vy(1,j)+ (a(2,j) + a(2,j+1))*imu.ts/2;
     Vz(1,j+1) =Vz(1,j)+ (a(3,j) + a(3,j+1))*imu.ts/2;
    end
end
figure;
subplot(311);plot(t',Vx',':','linewidth',1.5,'color',RGB('#333333'));hold on;
subplot(312);plot(t',Vy',':','linewidth',1.5,'color',RGB('#333333'));hold on;
subplot(313);plot(t',Vz',':','linewidth',1.5,'color',RGB('#333333'));hold on;
suptitle('仅对摆动期积分得到的速度')
%%
%偏差计算
Error_x = zeros(1,n);
Error_y = zeros(1,n);
Error_z = zeros(1,n);
for i=1:1:n
    Error_x(1,i) = Vx(1,arr_gait_time(2,i)+1)/((arr_gait_time(2,i)-arr_gait_time(1,i)+1)*imu.ts);
    Error_y(1,i) = Vy(1,arr_gait_time(2,i)+1)/((arr_gait_time(2,i)-arr_gait_time(1,i)+1)*imu.ts);
    Error_z(1,i) = Vz(1,arr_gait_time(2,i)+1)/((arr_gait_time(2,i)-arr_gait_time(1,i)+1)*imu.ts);
end
figure;
subplot(311);plot(Error_x',':','linewidth',1.5,'color',RGB('#333333'));hold on;
subplot(312);plot(Error_y',':','linewidth',1.5,'color',RGB('#333333'));hold on;
subplot(313);plot(Error_z',':','linewidth',1.5,'color',RGB('#333333'));hold on;
suptitle('ZVI处计算的速度误差')
%%
%剔除偏差后速度计算
Vx_RemoveErr = zeros(1,N);
Vy_RemoveErr = zeros(1,N);
Vz_RemoveErr = zeros(1,N);
for i=1:1:n
    for j=arr_gait_time(1,i):arr_gait_time(2,i)
        Vx_RemoveErr(1,j) = Vx(1,j) -Error_x(1,i) * imu.ts *(j-arr_gait_time(1,i));
        Vy_RemoveErr(1,j) = Vy(1,j) -Error_y(1,i) * imu.ts *(j-arr_gait_time(1,i));
        Vz_RemoveErr(1,j) = Vz(1,j) -Error_z(1,i) * imu.ts *(j-arr_gait_time(1,i));
     end
end

%     for j=1:12944
%          vz(j,1) = Vwaist(3,j) -ve* 0.01 *j;
%      end
% vz = zeros(12944,1);
figure;
subplot(311);plot(t',Vx_RemoveErr','-','linewidth',1,'color',RGB('#FF6666'));
subplot(312);plot(t',Vy_RemoveErr','-','linewidth',1,'color',RGB('#99CC66'));
subplot(313);plot(t',Vz_RemoveErr','-','linewidth',1,'color',RGB('#0099FF'));
suptitle('线性方式剔除误差后得到的速度')

% Vx_RemoveErr = Vx;
% Vy_RemoveErr = Vy;
% Vz_RemoveErr = Vz;
%%
%位置计算
% 修正速度后的位置
position_x= zeros(1,N);
position_y= zeros(1,N);
position_z= zeros(1,N);
% 仅将ZVI处速度设为零，摆动期未修正
position0_x= zeros(1,N);
position0_y= zeros(1,N);
position0_z= zeros(1,N);
% 完全未修正
position1_x= zeros(1,N);
position1_y= zeros(1,N);
position1_z= zeros(1,N);
% flag = 'foot';
for i=1:1:N-1

     position_x(1,i+1) = position_x(1,i) +   (Vx_RemoveErr(1,i)+ Vx_RemoveErr(1,i+1))* imu.ts/2;
     position_y(1,i+1) = position_y(1,i) +   (Vy_RemoveErr(1,i)+ Vy_RemoveErr(1,i+1))* imu.ts/2;
     position_z(1,i+1) = position_z(1,i) +   (Vz_RemoveErr(1,i)+ Vz_RemoveErr(1,i+1))* imu.ts/2;
     
     position0_x(1,i+1) = position0_x(1,i) +   (Vx(1,i)+ Vx(1,i+1))* imu.ts/2;
     position0_y(1,i+1) = position0_y(1,i) +   (Vy(1,i)+ Vy(1,i+1))* imu.ts/2;
     position0_z(1,i+1) = position0_z(1,i) +   (Vz(1,i)+ Vz(1,i+1))* imu.ts/2;
     
     position1_x(1,i+1) = position1_x(1,i) +   (V(1,i)+ V(1,i+1))* imu.ts/2;
     position1_y(1,i+1) = position1_y(1,i) +   (V(2,i)+ V(2,i+1))* imu.ts/2;
     position1_z(1,i+1) = position1_z(1,i) +   (V(3,i)+ V(3,i+1))* imu.ts/2;
    
end
% % 修正高度
% PError_z = zeros(1,n);
% for i=1:1:n
%      PError_z(1,i) = position_z(1,arr_gait_time(2,i)+1)/((arr_gait_time(2,i)-arr_gait_time(1,i)+1)*imu.ts);
% end
% Pz_RemoveErr = zeros(1,N);
% for i=1:1:n
%     for j=arr_gait_time(1,i):arr_gait_time(2,i)
%           Pz_RemoveErr(1,j) = position_z(1,j) -PError_z(1,i) * imu.ts *(j-arr_gait_time(1,i));
%      end
% end
% figure;hold on;plot(position_z,':k');plot(Pz_RemoveErr,'b');
figure
plot(position_x,position_y);hold on;
plot(position0_x,position0_y);
legend({'corrected','semi-corrected'});
figure
plot(position1_x,position1_y);
legend({'uncorrected'});

V_correct = [Vx_RemoveErr;Vy_RemoveErr;Vz_RemoveErr]';

pos = ([position_x;position_y;position_z]')*1;%kou_yun5 *1.055 新主楼小腿手动给零速率区间1.023 新主楼脚部1.033
e=(pos(2:end,:)-pos(1:end-1,:));
sum(sqrt(e(:,1).^2+e(:,2).^2))
% figure;plot(pos(:,1),pos(:,2))


end
