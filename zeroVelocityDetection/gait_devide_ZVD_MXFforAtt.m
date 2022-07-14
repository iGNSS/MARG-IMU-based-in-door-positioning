function [arr_gait_time,stationaryStart,stationaryEnd,stationary,Test_statistics,HS]=gait_devide_ZVD_MXFforAtt(imu,g)
%零速率检测
global glv;
config.detector_type = 'GLRT';%'MAG';% 'MV','GLRT'
config.window_size = 3;

% config.testStaThreshold = 320000;%1.5e5;

    config.testStaThreshold =1.5e5;

config.g = glv.g0;
config.sigma_a = 0.01;
config.sigma_g = 0.1/180*pi;
[stationary,~] = zeroVelocityDetector(imu,config);HS=[];Test_statistics = [];stationary = stationary';
%  [stationary,Test_statistics, HS] = stance_phase_detect(imu,g);%Yu Yutao

%站立期和摆动期划分
stationaryStart = find([0; diff(stationary)] == 1);%0变1
stationaryEnd = find([0; diff(stationary)] == -1);%1变0

%第i个start在第i个end后面
% n=length(stationaryStart);
% for i=1:n
%     if(stationaryStart(i,1)-stationaryEnd(i,1) < 50)
%         stationary(stationaryEnd(i,1):stationaryStart(i,1),1)=1;
%     end
% end
% stationaryStart = find([0; diff(stationary)] == 1);
% stationaryEnd = find([0; diff(stationary)] == -1);
n=length(stationaryStart);
% for i=2:n
%     if(stationaryEnd(i,1) -stationaryStart(i-1,1)< 5)
%         stationary(stationaryStart(i-1,1):stationaryEnd(i,1),1)=0;
%     end
% end
for i=1:n
    if(stationaryStart(i,1)-stationaryEnd(i,1) < 5)
        stationary(stationaryEnd(i,1):stationaryStart(i,1),1)=1;
    end
end
stationaryStart = find([0; diff(stationary)] == 1);
stationaryEnd = find([0; diff(stationary)] == -1);
arr_gait_time=[stationaryEnd,stationaryStart]';