function Cns = sSys2rfu(sSys,order)
% Calculate DCM from s-frame to n-frame in for a static situation.
% Notes:   r:right l:left f:front b:back u:up d:down
%              n-frame direction: rfu
% Prototype:   Cns = s2rfuStatic('lfu')
% Input:   sSys - directiono of s-frame, should be recorded
%                        manually while doing experiments.
%             order(optional) - order of rotation of  DCM construction
% Output:  DCM from s-frame to n-frame, vecotr_n = Cns*vecotr_s,
%               i.e. Cns = C^n_s

% Copyright(c) 2019-2022, by Xiaofeng Ma, All rights reserved.
% 29/04/2020, 21/07/2020, 31/05/2022

if isa(sSys,'double')
    att0 = sSys;
    sSys = 'defined';
end

switch sSys
    case 'rfu'
        Csn = eye(3);  
    case 'frd'
        Csn = dcm('x',pi)*dcm('z',pi/2);
    case 'luf'
        Csn = dcm('x',-pi/2)*dcm('y',pi);%or Csn = dcm('y',pi)*dcm('x',pi/2);
    case 'rub'
        Csn = dcm('x',pi/2);
%     case 'bul'
%         Csn = [0 0 1;0 1 0;-1 0 0]*[1 0 0;0 0 1;0 -1 0];
    case 'fur' 
        Csn = dcm('x',pi/2)*dcm('z',pi/2);
    case 'rdf'   
        Csn = dcm('x',-pi/2);
    case 'defined'
        Csn = (att2dcm(att0,order))';        
    otherwise
        error('Undefined s-frame');
end
Cns = Csn';
end
function C = dcm(axis, angle)
switch axis
    case 'x'
        C = [1 0 0;0 cos(angle) sin(angle);0 -sin(angle) cos(angle)];
    case 'y'
        C = [cos(angle) 0 -sin(angle);0 1 0;sin(angle) 0 cos(angle)];
    case 'z'
        C = [cos(angle) sin(angle) 0;-sin(angle) cos(angle) 0;0 0 1];
    otherwise
        error('axis wrong')
end
end