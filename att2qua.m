function qsn = att2qua(att,order)
% Convert Euler angles to attitude quaternion.
%
% Prototype: qnb = att2qua(att,'zyx')
% Input: att - att=[pitch; roll; yaw] in radians
% Output: qsn - quaternion from n to s

% Xiaofeng Ma

    att2 = att/2;
    s = sin(att2); c = cos(att2);
    sx = s(1); sy = s(2); sz = s(3); 
    cx = c(1); cy = c(2); cz = c(3); 
    
if strcmp(order,'zxy')
    qsn = [ cx*cy*cz - sx*sy*sz;  
            sx*cy*cz - cx*sy*sz;
            cx*sy*cz + sx*cy*sz;
            sx*sy*cz + cx*cy*sz];
elseif strcmp(order,'zyx')
    qsn = [ cx*cy*cz + sx*sy*sz;  
            sx*cy*cz - cx*sy*sz;
            cx*sy*cz + sx*cy*sz;
            cx*cy*sz - sx*sy*cz];
else
        error('undefined order');
        
end