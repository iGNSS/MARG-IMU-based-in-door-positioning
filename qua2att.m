function att = qua2att(q,order)
% Convert quaternion to Euler attitude angles.
% s-frame pointed to rfu.
% Prototype: att = qua2att(q)
% Input: q - attitude quaternion
%           order - rotation order
% Output: att - Euler angles att=[pitch; roll; yaw] in radians

% by Xiaofeng Ma
   
    Cbn = qua2dcm(q,'Cbn');
    
if strcmp(order,'zxy')      
    att = [asin(Cbn(2,3)), atan2(-Cbn(1,3),Cbn(3,3)), atan2(-Cbn(2,1),Cbn(2,2)) ];
elseif strcmp(order,'zyx')
    att = [atan2(Cbn(2,3),Cbn(3,3)), asin(-Cbn(1,3)), atan2(Cbn(1,2),Cbn(1,1)) ]; %sign??
else
    error('undefined order');
end

end
 