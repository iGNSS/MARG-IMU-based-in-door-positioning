 function [z_pre,z_obs,H] = h(state_type,X,obs_type,obs,para)
q0 = X(1); q1 = X(2); q2 = X(3); q3 = X(4);
z_obs = obs;
if strcmp(obs_type,'gn')
    % ### The output of accelerometers should be equal to the grivaty
    % ### expressed in the b frame.
    g = para(3);
    switch state_type
        case 'q'       
            z_pre = qua2dcm(X(1:4),'Cbn')*para;
            H = [-2*g*q2,  2*g*q3, -2*g*q0, 2*g*q1;
                 2*g*q1,  2*g*q0,  2*g*q3, 2*g*q2;
                 2*g*q0, -2*g*q1, -2*g*q2, 2*g*q3];
        case 'q bG'
            z_pre = qua2dcm(X(1:4),'Cbn')*para;
            H = [-2*g*q2,  2*g*q3, -2*g*q0, 2*g*q1, 0, 0, 0;
                 2*g*q1,  2*g*q0,  2*g*q3, 2*g*q2, 0, 0, 0;
                 2*g*q0, -2*g*q1, -2*g*q2, 2*g*q3, 0, 0, 0];
        case 'q bG bA'
            z_pre = qua2dcm(X(1:4),'Cbn')*para + X(8:10);
            H = [-2*g*q2,  2*g*q3, -2*g*q0, 2*g*q1, 0, 0, 0, 1, 0, 0;
                 2*g*q1,  2*g*q0,  2*g*q3, 2*g*q2, 0, 0, 0, 0, 1, 0;
                 2*g*q0, -2*g*q1, -2*g*q2, 2*g*q3, 0, 0, 0, 0, 0, 1];
                 
        otherwise
            error('wrong');
    end
elseif strcmp(obs_type,'hnx')
    % ### The output of magnetometers should be equal to 
else
end

 
end