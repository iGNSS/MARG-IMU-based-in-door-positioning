function Cns = att2dcm(att,order)
% Convert Euler angles to DCM.
% s-frame pointed to rfu.
% Prototype: Cns = att2dcm(att, 'zyx')
% Input: att - att=[pitch; roll; yaw], rad
% Output: Cns - DCM from s-frame to n-frame
%                      i.e. Cns = C^n_s
% Xiaofeng Ma

    s = sin(att); c = cos(att);
    si = s(1); sj = s(2); sk = s(3); 
    ci = c(1); cj = c(2); ck = c(3);
    
    if strcmp(order,'zxy')
      Cns = [ cj*ck-si*sj*sk, -ci*sk,  sj*ck+si*cj*sk;
            cj*sk+si*sj*ck,  ci*ck,  sj*sk-si*cj*ck;
           -ci*sj,           si,     ci*cj           ];
    elseif strcmp(order,'zyx')
        Csn = [cj*ck,        cj*sk,           -sj;
            si*sj*ck-ci*sk,  si*sj*sk+ci*ck,  si*cj;
           ci*sj*ck+si*sk,   ci*sj*sk-si*ck,  ci*cj];
       Cns = Csn';
    else
        error('Undefined order');
        
    end
       
       


