%function plotTestData()
% Reads a .csv file containing test data from the Rowdy Runner II.
% Calculates and plots pitch, setpoint, velocity, power, voltage, and
% current.
% close all
% clear all

%%%%%%%%% simulated data %%%%%%%
plotSimData;

PaperPosition = [-0.25 -0.1 8 6]; %location on printed page. rect = [left, bottom, width, height]
PaperSize = [7.25 5.8]; %[width height]
Fontsize = 12;
Linewidth = 1;
Linewidth2 = 2;
print_pdf = 1;
path = 'results/';
gearing = 4.5;
%%%%%%% http://learningrc.com/motor-kv/ 
%%%% kt = 1/kv
kt = (2*gearing)/( (280*2*pi)/60); %torque constant. 270 rpm/V = (270/60) rad/s/V; factor 2 is for two motors
n = 10;
angle_degrees = 360/n;



% [imgname,imgpath] = uigetfile('*.csv','Please select a data file',...
%                               'testdata.csv');
% 
% if imgname == 0
%     return
% else
%     img_file = strcat(imgpath,imgname);
%     dat = csvread(img_file);
% end

%filename = 'testdata01_50.csv';
%filename = 'WalkingData.csv';
%img_file = ['/Users/pranavb/Dropbox/work_in_progress/paper_rimless_wheel/eric_experiments/',filename];
img_file = 'WalkingData.csv';
% img_file = '/Users/pranavb/Dropbox/work_in_progress/paper_rimless_wheel/eric_experiments/testdata01_50.csv';
dat = csvread(img_file);


% Get the gains (first line) and then remove the line from the rest
gains_ = dat(1,:);
% Line 2 is table column label
dat = dat(2:end,:);
 
% Make the time relative to the first sample
%dat(:,1) = dat(:,1) - dat(1,1);
time = dat(:,1)-dat(1,1);
pitch_degrees = dat(:,2);
pitch_setpt_degrees = dat(:,3);
current = dat(:,4);
pitch_rate_degrees = -dat(:,5);
absolute_angular_speed_degrees = -dat(:,7)/gearing; 
%absolute_angle_degrees = dat(:,8);
absolute_angle_degrees = rem(dat(:,8)+180,angle_degrees)-angle_degrees/2;
bus_voltage = dat(:,9);
bus_current = dat(:,10);


%tstart = time(1);
%tend = time(end);
% tstart = 16.73;
% tend = 32.35;
% tstart = 4;
% tend = 24;
tstart = 4;
tend = 9;
index = find(time>tstart & time<tend);
new_time = time(index)-tstart;



% %%%%%%%% test code %%%%%%%%%
% figure(1)
% plot(new_time,deg2rad(absolute_angle_degrees(index)),'LineWidth',Linewidth);
% %plot(new_time,rem(absolute_angle_degrees(index)+180,angle_degrees),'LineWidth',Linewidth);
% ylabel('absolute angle (rad)','Fontsize',Fontsize);
% hi
% %%%%%%%%%%
disp('values from experiments');
i=1;
hh=figure(i); i=i+1;
plot(new_time,deg2rad(pitch_degrees(index)),'LineWidth',Linewidth); hold on;
mean_pitch = mean(pitch_degrees(index));
disp(['mean pitch = ',num2str(mean_pitch)]);
%line([new_time(1) new_time(end)],[mean_pitch mean_pitch],'LineWidth',Linewidth2,'Color','black');
%text(0.5*(new_time(1)+new_time(end)), mean_pitch+0.5,num2str(mean_pitch),'Fontsize',Fontsize,'Color','black');
ylabel('Torso pitch angle (degrees)','Fontsize',Fontsize);
xlabel('Time (s)','Fontsize',Fontsize);
ylim([0 max(get(gca,'ylim'))]);
xlim([0 5]);
string = [path,'pitch'];
set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
if (print_pdf==1)
       print(hh,'-dpdf',string);
end 

%figure(i); i=i+1;
%plot(new_time,pitch_setpt_degrees(index));

if (0)
hh=figure(i); i=i+1;
plot(new_time,current(index),'LineWidth',Linewidth); hold on;
mean_current = mean(current(index));
%line([new_time(1) new_time(end)],[mean_current mean_current],'LineWidth',Linewidth2,'Color','black');
%text(0.5*(new_time(1)+new_time(end)), mean_current+0.5,num2str(mean_current),'Fontsize',Fontsize,'Color','black');
ylabel('Current (A)','Fontsize',Fontsize);
xlabel('Time (s)','Fontsize',Fontsize);
string = [path,'current'];
set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
if (print_pdf==1)
       print(hh,'-dpdf',string);
end 
end

hh=figure(i); i=i+1;
plot(new_time,kt*current(index),'LineWidth',Linewidth); hold on;
mean_torque = kt*mean(current(index));
disp(['mean torque = ',num2str(mean_torque)]);
%line([new_time(1) new_time(end)],[mean_torque mean_torque],'LineWidth',Linewidth2,'Color','black');
%text(0.5*(new_time(1)+new_time(end)), mean_torque+0.5,num2str(mean_torque),'Fontsize',Fontsize,'Color','black');                                  
ylabel('Torque (Nm)','Fontsize',Fontsize);
xlabel('Time (s)','Fontsize',Fontsize);
xlim([0 5]);
string = [path,'torque'];
set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
if (print_pdf==1)
       print(hh,'-dpdf',string);
end 

if(0)
hh=figure(i); i=i+1;
plot(new_time,deg2rad(absolute_angle_degrees(index)),'LineWidth',Linewidth);
%plot(new_time,absolute_angle_degrees(index),'LineWidth',Linewidth);
ylabel('absolute angle (rad)','Fontsize',Fontsize);
string = [path,'absolute_angle'];
xlabel('time (s)','Fontsize',Fontsize);
set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
if (print_pdf==1)
       print(hh,'-dpdf',string);
end 
end


hh=figure(i); i=i+1;
plot(new_time,deg2rad(absolute_angular_speed_degrees(index)),'LineWidth',Linewidth); hold on;
mean_speed = mean(deg2rad(absolute_angular_speed_degrees(index)));
disp(['mean speed = ',num2str(mean_speed)]);
%line([new_time(1) new_time(end)],[mean_speed mean_speed],'LineWidth',Linewidth2,'Color','black');
%text(0.5*(new_time(1)+new_time(end)), mean_speed+0.5,num2str(mean_speed),'Fontsize',Fontsize,'Color','black');
ylabel('Absolute angular speed (rad/s)','Fontsize',Fontsize);
xlabel('Time (s)','Fontsize',Fontsize);
ylim([min(get(gca,'ylim')) 0]);
xlim([0 5]);
string = [path,'absolute_speed'];
set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
if (print_pdf==1)
       print(hh,'-dpdf',string);
end 

% hh=figure(i); i=i+1;
% plot(new_time,bus_voltage(index),'LineWidth',Linewidth);
% ylabel('bus voltages (V)','Fontsize',Fontsize);
% xlabel('time (s)','Fontsize',Fontsize);
% string = [path,'bus_voltage'];
% set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
% set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
% if (print_pdf==1)
%        print(hh,'-dpdf',string);
% end 

% hh=figure(i); i=i+1;
% plot(new_time,bus_current(index),'LineWidth',Linewidth);
% ylabel('bus current (A)','Fontsize',Fontsize);
% xlabel('time (s)','Fontsize',Fontsize);
% string = [path,'bus_current'];
% set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
% set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
% if (print_pdf==1)
%        print(hh,'-dpdf',string);
% end 

hh=figure(i); i=i+1;
power = abs(gearing*kt*deg2rad(absolute_angular_speed_degrees(index)));
%power = bus_current(index).*bus_voltage(index);
plot(new_time,power,'LineWidth',Linewidth); hold on;
mean_power = mean(power);
disp(['mean power = ',num2str(mean_power)]);
%line([new_time(1) new_time(end)],[mean_power mean_power],'LineWidth',Linewidth2,'Color','black');
%text(0.5*(new_time(1)+new_time(end)), mean_power+0.5,num2str(mean_power),'Fontsize',Fontsize,'Color','black');
ylabel('Power (W)','Fontsize',Fontsize);
xlabel('Time (s)','Fontsize',Fontsize);
ylim([0 max(get(gca,'ylim'))]);
xlim([0 5]);
string = [path,'power'];
set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
if (print_pdf==1)
       print(hh,'-dpdf',string);
end 

disp('values from actual data');
power_motor = mean(bus_voltage(index).*bus_current(index));
power_motor_alternate = mean(kt*current(index).*deg2rad(-absolute_angular_speed_degrees(index)));
disp(['average motor power = ',num2str(power_motor)]);
disp(['average motor power (alternate) = ',num2str(power_motor_alternate )]);
power_pi = 5;
power_teensy = 0.2;
power_total = power_motor_alternate + power_pi + power_teensy
mass = 6.9;
g = 9.81;
leg_length = 0.26;
speed_rad_s = -mean(deg2rad(absolute_angular_speed_degrees(index)));
speed_m_s = leg_length*speed_rad_s
COT = power_total/ (mass*g*speed_m_s)
