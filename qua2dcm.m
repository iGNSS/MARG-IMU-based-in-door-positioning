function Cnb = qua2dcm(q)
% Convert attitude quaternion to direction cosine matrix(DCM).
%
% Prototype: Cnb = qua2dcm(q)
% Input: qnb - attitude quaternion
% Output: Cnb - DCM from body-frame to navigation-frame
%
% See also  a2mat, a2qua, m2att, m2qua, q2att, attsyn.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/02/2008
    q11 = q(1)*q(1); q12 = q(1)*q(2); q13 = q(1)*q(3); q14 = q(1)*q(4); 
    q22 = q(2)*q(2); q23 = q(2)*q(3); q24 = q(2)*q(4);     
    q33 = q(3)*q(3); q34 = q(3)*q(4);  
    q44 = q(4)*q(4);
    Cnb = [ q11+q22-q33-q44,  2*(q23-q14),     2*(q24+q13);
            2*(q23+q14),      q11-q22+q33-q44, 2*(q34-q12);
            2*(q24-q13),      2*(q34+q12),     q11-q22-q33+q44 ];
