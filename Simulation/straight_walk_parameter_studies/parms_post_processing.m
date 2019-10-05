clc
close all
clear all

PaperPosition = [-0.25 -0.1 8 6]; %location on printed page. rect = [left, bottom, width, height]
PaperSize = [7.25 5.8]; %[width height]
Fontsize = 12;
Linewidth = 1;
Linewidth2 = 2;
print_pdf = 0;
path = 'results/';


%%%%%%%%%% run each of them to get the data saved into the folder /results/
%parameter = 'alpha'; %parms.control.alpha
%parameter = 'stiffness'; %parms.k 
%parameter = 'spokes'; %parms.n
%parameter = 'com'; %parms.c
parameter = 'mWheel'; %parms.m1

if (strcmp(parameter,'alpha'))
    title1 = 'parm_alpha.mat';
    load(title1);
    variable = alpha;
elseif (strcmp(parameter,'spokes'))
    title1 = 'parm_spokes.mat';
    load(title1);
    variable = spokes;
elseif (strcmp(parameter,'stiffness'))
     title1 = 'parm_stiffness.mat';
     load(title1);
     variable = k;
elseif (strcmp(parameter,'com'))    
     title1 = 'parm_com.mat';
     load(title1);
     variable = c;
elseif (strcmp(parameter,'mWheel'))    
     title1 = 'parm_mWheel.mat';
     load(title1);
     variable = mWheel;
else
    error('Incorrect parameter value');
end

hh=figure(1);
subplot(4,1,1)
plot(variable,robot_speed_all,'LineWidth',Linewidth2);
ylabel('Speed (m/s)','Fontsize',Fontsize);
subplot(4,1,2)
plot(variable,torque_all,'LineWidth',Linewidth2);
ylabel('Torque (Nm)','Fontsize',Fontsize);
subplot(4,1,3)
plot(variable,power_all,'LineWidth',Linewidth2);
ylabel('Power (W)','Fontsize',Fontsize);
subplot(4,1,4)
plot(variable,TCOT_motor_all,'LineWidth',Linewidth2);
ylabel('TCOT motor','Fontsize',Fontsize);
xlabel(parameter,'Fontsize',Fontsize);


if (strcmp(parameter,'com'))  
    for i=1:4
        subplot(4,1,i)
        xlim([0.02 0.2]);
    end
end

string = [path,parameter];
set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
if (print_pdf==1)
       print(hh,'-dpdf',string);
end 



% figure(1)
% plot(variable,power_all);
% ylabel('Power (W)');
% xlabel(parameter);
% 
% figure(2)
% plot(variable,torque_all);
% ylabel('Torque (Nm)');
% xlabel(parameter);
% 
% figure(3)
% plot(variable,TCOT_all);
% ylabel('TCOT');
% xlabel(parameter);
% 
% figure(3)
% plot(variable,TCOT_motor_all);
% ylabel('TCOT motor');
% xlabel(parameter);
% 
% figure(4) 
% plot(variable,robot_speed_all);
% ylabel('Speed (m/s)');
% xlabel(parameter);

