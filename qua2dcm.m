function C = qua2dcm(q,flag)
% Convert attitude quaternion to direction cosine matrix(DCM).
%
% Prototype: Cnb = qua2dcm(q)
% Input: qnb - attitude quaternion
% Output: C - DCM

    q11 = q(1)*q(1); q12 = q(1)*q(2); q13 = q(1)*q(3); q14 = q(1)*q(4); 
    q22 = q(2)*q(2); q23 = q(2)*q(3); q24 = q(2)*q(4);     
    q33 = q(3)*q(3); q34 = q(3)*q(4);  
    q44 = q(4)*q(4);
    if strcmp(flag,'Cnb') % DCM from body-frame to navigation-frame
    C = [ q11+q22-q33-q44,  2*(q23-q14),     2*(q24+q13);
            2*(q23+q14),      q11-q22+q33-q44, 2*(q34-q12);
            2*(q24-q13),      2*(q34+q12),     q11-q22-q33+q44 ];
    elseif strcmp(flag,'Cbn') % DCM from navigation-frame to body-frame
    C = [ q11+q22-q33-q44,  2*(q23+q14),     2*(q24-q13);
            2*(q23-q14),      q11-q22+q33-q44, 2*(q34+q12);
            2*(q24+q13),      2*(q34-q12),     q11-q22-q33+q44 ];        
    else
        error('Wrong')
    end
