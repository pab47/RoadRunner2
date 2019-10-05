function plotTestData()
% Reads a .csv file containing test data from the Rowdy Runner II.
% Calculates and plots pitch, setpoint, velocity, power, voltage, and
% current.
close all
[imgname,imgpath] = uigetfile('*.csv','Please select a data file',...
                              'testdata.csv');

if imgname == 0
    return
else
    img_file = strcat(imgpath,imgname);
    dat = csvread(img_file);
end

% Get the gains (first line) and then remove the line from the rest
gains_ = dat(1,:);
% Line 2 is table column label
dat = dat(2:end,:);
 
% Make the time relative to the first sample
dat(:,1) = dat(:,1) - dat(1,1);

% % Map to +/- 18
% if (min(dat(:,8)) >= -180) && (max(dat(:,8)) <= 180)
%     % Create empty var
%     boundangle = zeros(length(dat(:,8)),1);
%     
%     for i = 1:length(dat(:,8))
%         
%         % Convert to 360
%         if dat(i,8) < 0
%             dat(i,8) = dat(i,8) + 360;
%         end
%         
%         % Convert to +/- 18
%         segment = mod(dat(i,8),36);
%         
%         if segment <= 18
%             boundangle(i) = segment;
%         elseif segment > 18
%             boundangle(i) = segment - 36;
%         end
%     end
% end

% Plot pitch and setpoint
figure
plot(dat(:,1), dat(:,2:3))
xlim([0 max(dat(:,1))]);
ylim([0 max(max(dat(:,2:3)))+5]);
legend('Pitch','Setpoint','Location','best')
xlabel('Time (s)')
ylabel('Degrees')
gains = strcat(' Kp = ', num2str(gains_(1)), ', Ki = ', num2str(gains_(2)),...
    ', Kd = ', num2str(gains_(3)));
title(strcat('Pitch and Setpoint Over Time: ', gains))

% Plot Odrive Data
figure
yyaxis left
plot(dat(:,1),dat(:,4))
xlim([0 max(dat(:,1))]);
ylim([0 max(dat(:,4))+2]);
% legend('Pitch','Setpoint')
xlabel('Time (s)')
ylabel('Current (A)')

yyaxis right
plot(dat(:,1),dat(:,2:3))
ylim([0 max(max(dat(:,2:3)))+5]);
legend('Pitch','Setpoint','Current','Location','best')
ylabel('Degrees')

title('ODrive Input Over Time')

% Plot Pitch Rate
figure
plot(dat(:,1), dat(:,5))
xlim([0 max(dat(:,1))]);
ylim([min(dat(:,5))-10 max(dat(:,5))+10]);
legend('Pitch Rate')
xlabel('Time (s)')
ylabel('Degrees/s')
gains = strcat(' Kp = ', num2str(gains_(1)), ', Ki = ', num2str(gains_(2)),...
    ', Kd = ', num2str(gains_(3)));
title(strcat('Pitch Rate Over Time: ', gains))

% Plot Pitch and Pitch Rate
figure
yyaxis left
plot(dat(:,1),dat(:,2))
xlim([0 max(dat(:,1))]);
ylim([0 max(dat(:,2))+5]);
xlabel('Time (s)')
ylabel('Degrees')

yyaxis right
plot(dat(:,1),dat(:,5))
ylim([min(dat(:,5))-10 max(dat(:,5))+10]);
legend('Pitch','Pitch Rate','Location','best')
ylabel('Degrees/s')

% Absolute Position and Velocity
figure
subplot(211)
plot(dat(:,1),dat(:,8))
xlim([0 max(dat(:,1))])
ylim([0 max(dat(:,8))+5])
xlabel('Time (s)')
ylabel('Degrees')
title('Absolute Position')

subplot(212)
%plot(dat(:,1),dat(:,7))
plot(dat(:,1),dat(:,7))
xlim([0 max(dat(:,1))])
ylim([min(dat(:,7))-5 max(dat(:,7))+5])
ylabel('Degrees/s')

title('Velocity Over Time')

% Velocity Radians
figure
velrad = deg2rad(dat(:,7))./4.5;
plot(dat(:,1), velrad)
xlim([0 max(dat(:,1))])
ylim([min(velrad)-2 max(velrad)+2])
xlabel('Time (s)')
ylabel('Rad/s')
title('Velocity Rad')

% % Absolute Position
% figure
% plot(dat(:,1),boundangle)
% xlim([0 max(dat(:,1))])
% ylim([min(boundangle)-5 max(boundangle)+5])
% xlabel('Time (s)')
% ylabel('Degrees')
% title('Bound Absolute Position')

% Bus Voltage and Motor Current
figure
yyaxis left
plot(dat(:,1),dat(:,9))
xlim([0 max(dat(:,1))]);
ylim([0 max(dat(:,9))+5]);
xlabel('Time (s)')
ylabel('Voltage (V)')

yyaxis right
plot(dat(:,1),dat(:,10))
ylim([min(dat(:,10))-2 max(dat(:,10))+2]);
ylabel('Current (A)')
legend('Voltage','Current')

title('ODrive Input Over Time')

% Power
figure
power = dat(:,9).*dat(:,10);
plot(dat(:,1), power)
xlim([0 max(dat(:,1))]);
ylim([min(power)-5 max(power)+5]);
xlabel('Time (s)')
ylabel('Power (W)')
gains = strcat(' Kp = ', num2str(gains_(1)), ', Ki = ', num2str(gains_(2)),...
    ', Kd = ', num2str(gains_(3)));
title(strcat('Power Over Time: ', gains))

% % Pitch vs Phi
% figure
% plot(dat(:,1), dat(:,2),dat(:,1), rad2deg(dat(:,11)))
% xlim([0 max(dat(:,1))]);
% ylim([0 max(dat(:,2))+5]);
% legend('Pitch','Setpoint','Location','best')
% xlabel('Time (s)')
% ylabel('Degrees')
% gains = strcat(' Kp = ', num2str(gains_(1)), ', Ki = ', num2str(gains_(2)),...
%     ', Kd = ', num2str(gains_(3)));
% title(strcat('Pitch and Phi (Setpoint) Over Time: ', gains))
% for i=1:8
%     if (i~=6)
%         close(figure(i))
%     end
% end

