format long
close all
clc
clear all

PaperPosition = [-0.25 -0.1 8 6]; %location on printed page. rect = [left, bottom, width, height]
PaperSize = [7.25 5.8]; %[width height]
Fontsize = 12;
Linewidth = 1;
Linewidth2 = 2;
print_pdf = 0;
path = 'results/';

parms = get_parms;
qL = 0;%angle should always be zero
uL = -3.05; %-3.5; %-0.5%mid-stance velocity 
qR = qL+(pi/parms.n); 
uR = uL; 
xmid0 = 0;
ymid0 = 0;
phi0 = 0;
fps = 4;
total_time = 1.5;


q2 = parms.control.alpha; 
u2 = 0;
l0 = parms.l0;
l0dot = 0;
e_q2 = 0;
z0 = [qL uL qR uR q2 u2 l0 l0dot xmid0 ymid0 phi0 e_q2];% [angle rate];
[z,t,steps,T2,dT2,error_flag] = forward_dynamics(z0,parms,total_time);
disp('x    y     phi');
disp([z(end,9) z(end,10) z(end,11)*(180/pi)]);
%Dist_x = z(end,9)
%Speed_x = z(end,9)/t(end)
disp(['Number of steps is ',num2str(steps)]);    
    disp('Animating...');
    


AZ = -62; EL = 26;
%AZ = 0; EL = 0;
%AZ = -90; EL = 90;
%AZ = -44; EL = 16;
view_angle = [AZ EL];


save('sim_data.mat','t','z','T2','dT2');

hh=figure(1);
subplot(2,1,1)
plot(t,z(:,1),'Linewidth',Linewidth); hold on;
plot(t,z(:,3),'Linewidth',Linewidth2);
ylabel('absolute angles (rad)','Fontsize',Fontsize);
legend('left','right');
subplot(2,1,2)
plot(t,z(:,2),'Linewidth',Linewidth); hold on;
plot(t,z(:,4),'Linewidth',Linewidth2); 
ylabel('absolute angular rates (rad/s)','Fontsize',Fontsize);
legend('left','right');
xlabel('time (s)','Fontsize',Fontsize);
string = [path,'wheel'];
set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
if (print_pdf==1)
       print(hh,'-dpdf',string);
end 

hh=figure(2);
subplot(2,1,1);
plot(t,z(:,5)*(180/pi),'k','Linewidth',Linewidth); hold on;
ylabel('torso angle (deg)');
subplot(2,1,2);
plot(t,z(:,6),'k','Linewidth',Linewidth); hold on;
ylabel('torso speed (rad/s)','Fontsize',Fontsize);
xlabel('time (s)','Fontsize',Fontsize);
string = [path,'torso'];
set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
if (print_pdf==1)
       print(hh,'-dpdf',string);
end 

disp('mean torque on torso = '); 
disp(num2str(mean(T2)));
disp('mean speed = ')
disp(['left = ', num2str(mean(z(:,2))),';',' right = ', num2str(mean(z(:,4)))]);
disp('mean power = ')
disp(['left = ', num2str(mean((0.5*T2+dT2).*z(:,2))),';',' right = ', num2str(mean((0.5*T2-dT2).*z(:,4)))]);


hh=figure(3);
subplot(2,1,1)
plot(t,0.5*T2+dT2,'Linewidth',Linewidth); hold on;
plot(t,0.5*T2-dT2,'Linewidth',Linewidth2);
legend('left','right');
ylabel('torque (Nm)');
subplot(2,1,2)
plot(t,abs(0.5*(T2+dT2).*z(:,2)),'Linewidth',Linewidth); hold on;
plot(t,abs(0.5*(T2-dT2).*z(:,4)),'Linewidth',Linewidth2); 
legend('left','right');
ylabel('power (W)','Fontsize',Fontsize);
xlabel('time (s)','Fontsize',Fontsize);
string = [path,'power'];
set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
if (print_pdf==1)
       print(hh,'-dpdf',string);
end 

hh=figure(4);
plot(-z(:,9),-z(:,10),'k','Linewidth',Linewidth2); hold on;
xlabel('x','Fontsize',Fontsize);
ylabel('y','Fontsize',Fontsize);
axis('equal');
axis([0 1 -1 0]);
grid on;
string = [path,'cartesian'];
set(gcf, 'PaperPosition', PaperPosition); %Position the plot further to the left and down. Extend the plot to fill entire paper.
set(gcf, 'PaperSize', PaperSize); %Keep the same paper size
if (print_pdf==1)
       print(hh,'-dpdf',string);
end 

figure(5)
animation3D(t,z,parms,fps,steps,view_angle);
close(figure(5))
