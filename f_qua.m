function [X_pre,F] = f_qua(X,para,discrete_way,state_type)

wx = para(1); wy = para(2); wz = para(3);
Ts = para(4);

if strcmp(discrete_way,'Eular')
% X_k+1 = X_k + T*f(X_k);
    switch state_type
        case 'q' % states are quaternion
            X_pre = X + Ts*0.5*[0, -wx, -wy, -wz;
                               wx, 0, wz, -wy;
                               wy, -wz, 0, wx;
                     	       wz, wy, -wx, 0;]*X;
            F = [1, -(Ts*wx)/2, -(Ts*wy)/2, -(Ts*wz)/2;
                (Ts*wx)/2,          1,  (Ts*wz)/2, -(Ts*wy)/2;
                (Ts*wy)/2, -(Ts*wz)/2,          1,  (Ts*wx)/2;
                (Ts*wz)/2,  (Ts*wy)/2, -(Ts*wx)/2,          1];
            
        case 'q bG' % states are quaternion and gyroscope bias
            q0 = X(1); q1 = X(2); q2 = X(3); q3 = X(4);
            bGx = X(5); bGy = X(6); bGz = X(7);
            X_pre = X + Ts*0.5*[[0, -wx+bGx, -wy+bGy, -wz+bGz;
                               wx-bGx, 0, wz-bGz, -wy+bGy;
                               wy-bGy, -wz+bGz, 0, wx-bGx;
                     	       wz-bGz, wy-bGy, -wx+bGx, 0;]*X(1:4);
                               [0;0;0].*X(5:7)]; 
            F = [1,  (Ts*(bGx - wx))/2,  (Ts*(bGy - wy))/2,  (Ts*(bGz - wz))/2,  (Ts*q1)/2,  (Ts*q2)/2,  (Ts*q3)/2;
                -(Ts*(bGx - wx))/2,                  1, -(Ts*(bGz - wz))/2,  (Ts*(bGy - wy))/2, -(Ts*q0)/2,  (Ts*q3)/2, -(Ts*q2)/2;
                -(Ts*(bGy - wy))/2,  (Ts*(bGz - wz))/2,                  1, -(Ts*(bGx - wx))/2, -(Ts*q3)/2, -(Ts*q0)/2,  (Ts*q1)/2;
                -(Ts*(bGz - wz))/2, -(Ts*(bGy - wy))/2,  (Ts*(bGx - wx))/2,                  1,  (Ts*q2)/2, -(Ts*q1)/2, -(Ts*q0)/2;
                                 0,                  0,                  0,                  0,          1,          0,          0;
                                 0,                  0,                  0,                  0,          0,          1,          0;
                                 0,                  0,                  0,                  0,          0,          0,          1];
        
        case 'q bG bA' % states are quaternion, gyroscope bias, and accelerometer bias
            q0 = X(1); q1 = X(2); q2 = X(3); q3 = X(4);
            bGx = X(5); bGy = X(6); bGz = X(7);
            X_pre = X + Ts*0.5*[[0, -wx+bGx, -wy+bGy, -wz+bGz;
                               wx-bGx, 0, wz-bGz, -wy+bGy;
                               wy-bGy, -wz+bGz, 0, wx-bGx;
                     	       wz-bGz, wy-bGy, -wx+bGx, 0;]*X(1:4); 
                               [0;0;0;0;0;0].*X(5:10)];
            F = [1,  (Ts*(bGx - wx))/2,  (Ts*(bGy - wy))/2,  (Ts*(bGz - wz))/2,  (Ts*q1)/2,  (Ts*q2)/2,  (Ts*q3)/2, 0, 0, 0;
                -(Ts*(bGx - wx))/2,                  1, -(Ts*(bGz - wz))/2,  (Ts*(bGy - wy))/2, -(Ts*q0)/2,  (Ts*q3)/2, -(Ts*q2)/2, 0, 0, 0;
                -(Ts*(bGy - wy))/2,  (Ts*(bGz - wz))/2,                  1, -(Ts*(bGx - wx))/2, -(Ts*q3)/2, -(Ts*q0)/2,  (Ts*q1)/2, 0, 0, 0;
                -(Ts*(bGz - wz))/2, -(Ts*(bGy - wy))/2,  (Ts*(bGx - wx))/2,                  1,  (Ts*q2)/2, -(Ts*q1)/2, -(Ts*q0)/2, 0, 0, 0;
                                 0,                  0,                  0,                  0,          1,          0,          0, 0, 0, 0;
                                 0,                  0,                  0,                  0,          0,          1,          0, 0, 0, 0;
                                 0,                  0,                  0,                  0,          0,          0,          1, 0, 0, 0;
                                 0,                  0,                  0,                  0,          0,          0,          0, 1, 0, 0;
                                 0,                  0,                  0,                  0,          0,          0,          0, 0, 1, 0;
                                 0,                  0,                  0,                  0,          0,          0,          0, 0, 0, 1];
        otherwise
            error('not support');
    end
elseif strcmp(discrete_way,'exp')
% Are we assuming this is TI with in T? othewise it is a TV sys and this discretization doesn't work.    
% X_k+1 = expm(AT)*X_k;
    switch state_type
        case 'q'
            X_pre = expm(Ts*0.5*[0, -wx, -wy, -wz;
                                wx, 0, wz, -wy;
                                wy, -wz, 0, wx;
                                wz, wy, -wx, 0;])*X; 
        case 'q bG'
            bGx = X(5); bGy = X(6); bGz = X(7);
            X_pre = expm(Ts*0.5*[[0, -wx+bGx, -wy+bGy, -wz+bGz;
                                wx-bGx, 0, wz-bGz, -wy+bGy;
                                wy-bGy, -wz+bGz, 0, wx-bGx;
                                wz-bGz, wy-bGy, -wx+bGx, 0;],zeros(4,3);
                                zeros(3,7)])*X; 
        case 'q bG bA'
            bGx = X(5); bGy = X(6); bGz = X(7);
            X_pre = expm(Ts*0.5*[[0, -wx+bGx, -wy+bGy, -wz+bGz;
                               wx-bGx, 0, wz-bGz, -wy+bGy;
                               wy-bGy, -wz+bGz, 0, wx-bGx;
                     	       wz-bGz, wy-bGy, -wx+bGx, 0;],zeros(4,6);
                               zeros(3,10)])*X; 
        otherwise
            error('not support');
    end
else
    error('discrete method not support');
end

end