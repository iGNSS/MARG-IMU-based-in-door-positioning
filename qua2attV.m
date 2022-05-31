function att = qua2attV(q,order)
% Input: q - quaternion
% Output: att - attitude angles, rad

% Copyright(c) 2019-2020, by Xiaofeng Ma, All rights reserved.
% 22/07/2020

N = length(q);
att = zeros(N,3);
for i = 1:N
    att(i,:) = qua2att(q(i,:),order);
end

end

