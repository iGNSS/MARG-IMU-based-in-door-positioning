function [q,att]=alignbyAccMagatStatic(Acc, Mag, order)
% Calulate horizontal angles from accelerometers
% and initial yaw angle from magnetometers.
% s-frame pointed to rfu.
% Inputs:  Acc - outputs of accelerometers (under s-frame)
%              Mag - outputs of magnetometers  (under s-frame)
%              order - order of rotation of  DCM construction
% Output: att - Eular attitude
%               q - quarternion

% Copyright(c) 2019-2022, by Xiaofeng Ma, All rights reserved.
% 29/04/2020, 31/05/2022

fx=mean(Acc(:,1));
fy=mean(Acc(:,2));
fz=mean(Acc(:,3));

yaw=0;
switch order
    case 'zyx'
        pitch=atan2(fy,fz); 
        roll=asin(-fx/norm([fx,fy,fz])); 
    case 'zxy'
        pitch=asin(fy/norm([fx,fy,fz])); 
        roll=atan2(-fx,fz); 
    otherwise
        error('undefined order');
end


if ~isempty(Mag)
%     phi=-6.95*pi/180; % �����Ĵ�ƫ��/rad
    phi=0; 
    mx=mean(Mag(:,1));
    my=mean(Mag(:,2));
    mz=mean(Mag(:,3));
    
switch order
    case 'zyx'
   yaw=atan2(my*cos(pitch)-mz*sin(pitch),...
       mx*cos(roll)+my*sin(pitch)*sin(roll)+mz*cos(pitch)*sin(roll))+phi;       
    case 'zxy'
   yaw=atan2(mx*sin(roll)*sin(pitch)+my*cos(pitch)-mz*cos(roll)*sin(pitch),...
       mx*cos(roll)+mz*sin(roll))+phi;      
    otherwise
        error('undefined order');
end
end

att=[pitch,roll,yaw]';
q=att2qua([pitch roll yaw], order);

end