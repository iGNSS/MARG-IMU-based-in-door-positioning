function  [pos,V_correct,arr_gait_time,stationaryStart,stationaryEnd,stationary,Test_statistics]=posCalculateWithGait(imu,a)
%����imu: ������ٶȡ�������ٶȡ������ǿ��С
%����a:�޳��������������ٶ�
%���position_x,position_y,position_z��λ�õ���ά����
%���arr_gait_time��վ���ڿ�ʼ�ͽ�����������ɵľ���
%���stationaryStart,stationaryEnd��վ���ڽ����Ϳ�ʼ�Ĳ�����
%���stationary�������İڶ��ڲ������վ���ڲ�����
%���Test_statistics�����ٶȺͽ��ٶȹ�ͬ�����ͳ����

%��̬���ڻ���
global glv;
g=glv.g0;
% imu = foot_imu1;a = foot_accn';
[arr_gait_time,stationaryStart,stationaryEnd,stationary,Test_statistics,~]=gait_devide_ZVD_MXF(imu,g);
% figure;plot(foot_imu1.gyros(:,1));hold on;plot(stationary);
N=length(a);
n=length(arr_gait_time);
t = 0:imu.ts:imu.ts*(length(imu.acc)-1);
%%
%�ٶȼ���
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
suptitle('���������ݻ��ֵõ����ٶ�')
for i=1:1:n
    for j=arr_gait_time(1,i):arr_gait_time(2,i)%�԰ڶ��ڽ��л���
     Vx(1,j+1) =Vx(1,j)+ (a(1,j) + a(1,j+1))*imu.ts/2;
     Vy(1,j+1) =Vy(1,j)+ (a(2,j) + a(2,j+1))*imu.ts/2;
     Vz(1,j+1) =Vz(1,j)+ (a(3,j) + a(3,j+1))*imu.ts/2;
    end
end
figure;
subplot(311);plot(t',Vx',':','linewidth',1.5,'color',RGB('#333333'));hold on;
subplot(312);plot(t',Vy',':','linewidth',1.5,'color',RGB('#333333'));hold on;
subplot(313);plot(t',Vz',':','linewidth',1.5,'color',RGB('#333333'));hold on;
suptitle('���԰ڶ��ڻ��ֵõ����ٶ�')
%%
%ƫ�����
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
suptitle('ZVI��������ٶ����')
%%
%�޳�ƫ����ٶȼ���
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
suptitle('���Է�ʽ�޳�����õ����ٶ�')

% Vx_RemoveErr = Vx;
% Vy_RemoveErr = Vy;
% Vz_RemoveErr = Vz;
%%
%λ�ü���
% �����ٶȺ��λ��
position_x= zeros(1,N);
position_y= zeros(1,N);
position_z= zeros(1,N);
% ����ZVI���ٶ���Ϊ�㣬�ڶ���δ����
position0_x= zeros(1,N);
position0_y= zeros(1,N);
position0_z= zeros(1,N);
% ��ȫδ����
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
% % �����߶�
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

pos = ([position_x;position_y;position_z]')*1;%kou_yun5 *1.055 ����¥С���ֶ�������������1.023 ����¥�Ų�1.033
e=(pos(2:end,:)-pos(1:end-1,:));
sum(sqrt(e(:,1).^2+e(:,2).^2))
% figure;plot(pos(:,1),pos(:,2))


end
