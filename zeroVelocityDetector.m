function [zvd,testSta] = zeroVelocityDetector(imu,zvdConfig)
% funtion [zvd testSta] = zeroVelocityDector(imu)
%
% @brief Functions for zero-velocity detection with different algorithms
% for choosing.
%
% @param[out] zvd          Vector with the detector decsions. 
%   [ true = zero velocity, false = moving] 
% @param[out] testSta      The test statistics of the detector.It is an
% vector.
% @param[in]  imu          The IMU data structure.
%   imu.acc, imu.gyros, Unit: m/s^2,rad/s
% @param[in]  zvdConfig       Structure holding the dectect setting. 
%   zvdConfig.detector_type = 'GLRT','MV','MAG' or 'ARE'.
%   zvdConfig.window_size, chose the hypothesis that the system has zero velocity
%   zvdConfig.testStaThreshold, the detect threshold of the test statistic
% for zero velocity decision
%   zvdConfig.g
%   zvdConfig.sigma_a
%   zvdConfig.sigma_g
%

% Allocate memory
zvd = zeros(1,length(imu.acc));
% Run the desired detector type. Each detector return a vector with their
% calculated test statistic testSta.
switch zvdConfig.detector_type
    case 'GLRT'
        testSta = GLRT(imu,zvdConfig);
    case 'MV'
        testSta = MV(imu,zvdConfig);
    case 'MAG'
        testSta = MAG(imu,zvdConfig);
    case 'ARE'
        testSta = ARE(imu,zvdConfig);
    otherwise
        dis('The choosen detector type not recognized. The GLRT detector is used');
        testSta = GLRT(imu,zvdConfig);
end
% Check if the test statistics testSta are below the dectect threshold. If
% so, choose the hypothesis that the system has zero velocity.
window_size = zvdConfig.window_size;
for i = 1:length(testSta)
    if testSta(i)<zvdConfig.testStaThreshold
        zvd(i:i+window_size-1) = ones(1,window_size);
    end
end
% Fix the edges of the test statistics of the detectro.
testSta = [max(testSta)*ones(1,floor(window_size/2)) testSta max(testSta)*ones(1,floor(window_size/2))];
end
%% SUBFUNCTIONS
function testSta = GLRT(imu,zvdConfig)
% function testSta = GLRT(imu)
% 
% @brief Fuction that runs the generalized likelihood test (SHOE detector)
%
% @param[out] testSta       The test statistics of the detector.It is an
% vector.
% @param[in]  imu          The IMU data structure.
%   imu.acc, imu.gyros. Unit: m/s^2,rad/s
% @param[in]  zvdConfig       Structure holding the dectect setting. 
%   zvdConfig.detector_type = 'GLRT','MV','MAG' or 'ARE'.
%   zvdConfig.window_size, chose the hypothesis that the system has zero velocity
%   zvdConfig.testStaThreshold, the detect threshold of the test statistic
% for zero velocity decision
%   zvdConfig.g, Magnitude fo the local gravity vector [m/s^2], decide by
% gravity mode.
%   zvdConfig.sigma_a£¬standard deviation of the accelerometer noise
% [m/s^2].This is used to control the zero-velocity dectectors trust in
% the accelerometer data. Reference value zvdConfig.sigma_a = 0.01,adjust
% according to the noise level of the accelerometer.
%   zvdConfig.sigma_g, standard deviation of the gyroscope noise [rad/s]. This
% is used to control the zero-velocity detectors trust in the gyroscope
% data. Reference value zvdConfig.sigma_g = 0.1/180*pi,adjust
% according to the noise level of the gyroscope.
% 
    g = zvdConfig.g;
    sigma2_a = zvdConfig.sigma_a^2;
    sigma2_g = zvdConfig.sigma_g^2;
    window_size = zvdConfig.window_size;

    lengthImuData = length(imu.acc);
    testSta = zeros(1,lengthImuData-window_size+1);

    for i = 1:lengthImuData-window_size+1
        ya_m = mean(imu.acc(i:i+window_size-1,:))';% covert to colum vector
        for j = i:i+window_size-1
           tmp = imu.acc(j,:)' - g*ya_m/norm(ya_m);
           testSta(i) = testSta(i)+imu.gyros(j,:)*imu.gyros(j,:)'/sigma2_g + tmp'*tmp/sigma2_a;
        end
    end
    testSta = testSta/window_size;
end

function testSta = MV(imu,zvdConfig)
%
% @brife Function that runs the acceleration moving variance detector.
% 
% @param[out] testSta       The test statistics of the detector.It is an
% vector.
% @param[in]  imu          The IMU data structure.
%   imu.acc, imu.gyros,Unit: m/s^2,rad/s
% @param[in]  zvdConfig,      same as parameter zvdConfig in function GLRT().
%
    sigma2_a = zvdConfig.sigma_a^2;
    window_size = zvdConfig.window_size;

    lengthImuData = length(imu.acc);
    testSta = zeros(1,lengthImuData-window_size+1);

    for i = 1:lengthImuData-window_size+1
        ya_m = mean(imu.acc(i:i+window_size-1,:))';
        for j = i:i+window_size-1
            tmp = imu.acc(j,:)'-ya_m;
            testSta(i) = testSta(i)+tmp'*tmp;
        end
    end
    testSta = testSta/(sigma2_a*window_size);
end

function testSta = MAG(imu,zvdConfig)
%
% @brief Function that runs the acceleration magnitude detector.
%
% @param[out] testSta       The test statistics of the detector.It is an
% vector.
% @param[in]  imu          The IMU data structure.
%   imu.acc, imu.gyros,Unit: m/s^2,rad/s
% @param[in]  zvdConfig,      same as parameter zvdConfig in function GLRT().
%
    g = zvdConfig.g;
    sigma2_a = zvdConfig.sigma_a^2;
    window_size = zvdConfig.window_size;
    
    lengthImuData = length(imu.acc);  
    testSta = zeros(1,lengthImuData-window_size+1);
    
    for i = 1:lengthImuData-window_size+1
        for j = i:i+window_size-1
            testSta(i) = testSta(i)+(norm(imu.acc(j,:))-g)^2;
        end
    end
    testSta = testSta/(sigma2_a*window_size);
end

function testSta = ARE(imu,zvdConfig)
%
% @brief Function that runs the angular rate energy detector.
%
% @param[out] testSta       The test statistics of the detector.It is an
% vector.
% @param[in]  imu          The IMU data structure.
%   imu.acc, imu.gyros,Unit: m/s^2,rad/s.
% @param[in]  zvdConfig,      same as parameter zvdConfig in function GLRT().
%
    sigma2_g = zvdConfig.sigma_g^2;
    window_size = zvdConfig.window_size;

    lengthImuData = length(imu.acc);
    testSta = zeros(1,lengthImuData-window_size+1);

    for i = 1:lengthImuData-window_size+1
        for j = i:i+window_size-1
            testSta(i) = testSta(i)+norm(imu.gyros(j,:))^2;
        end
    end
    testSta = testSta/(sigma2_g*window_size);
end

        
        