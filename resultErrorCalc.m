marker = [0, 0;0,1.2;0, 2.4;0, 3.6;0, 4.8;0, 6.0;0, 7.2;0, 8.4;0, 9.6;
    -1.2, 9.6;
    -2.4, 9.6;-2.4, 8.4;-2.4,7.2;-2.4,6.0;-2.4,4.8;-2.4,3.6;-2.4, 2.4;-2.4, 1.2;-2.4, 0;
    -1.2, 0;0, 0];
figure
plot(marker(:,1),marker(:,2),'*-')

path_length = sum(sqrt(sum((marker(2:end,:)-marker(1:end-1,:)).^2,2)));

% plot(gait_time(1,:),1,'*')
% plot(gait_time(2,:),1,'p')

MS = round((gait_time(1,2:end)-gait_time(2,1:end-1))/2)+gait_time(2,1:end-1);
% plot(MS,1,'v')
FF = [1 MS gait_time(2,end)+5];
pos_FF = pos(FF,1:2);

error_pp = sqrt(sum((pos_FF-marker).^2,2));
error_mean = mean(error_pp);
error_mean/path_length

figure(900);hold on
plot(error_pp)