format long
close all
clc
clear all

parms = get_parms;
qL = 0;%angle should always be zero
uL = -3.5; %-0.5%mid-stance velocity 
qR = qL+(pi/parms.n); 
uR = uL; 
xmid0 = 0;
ymid0 = 0;
phi0 = 0;
fps = 4;
total_time = 5;


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
% 

save('sim_data.mat','t','z','T2','dT2');

figure(1)
subplot(2,1,1)
plot(t,z(:,1),t,z(:,3));
ylabel('absolute angles (rad)');
legend('left','right');
subplot(2,1,2)
plot(t,z(:,2),t,z(:,4));
ylabel('absolute angular rates (rad/s)');
legend('left','right');
xlabel('time (s)');

figure(2)
subplot(2,1,1);
plot(t,z(:,5)*(180/pi));
ylabel('torso angle (deg)');
subplot(2,1,2);
plot(t,z(:,6));
ylabel('torso speed rad/s');
xlabel('time (s)');


disp('mean torque on torso = '); 
disp(num2str(mean(T2)));
disp('mean speed = ')
disp(['left = ', num2str(mean(z(:,2))),';',' right = ', num2str(mean(z(:,4)))]);
disp('mean power = ')
disp(['left = ', num2str(mean((0.5*T2+dT2).*z(:,2))),';',' right = ', num2str(mean((0.5*T2-dT2).*z(:,4)))]);


figure(3)
subplot(2,1,1)
plot(t,T2+dT2,t,T2-dT2);
legend('left','right');
ylabel('torque (Nm)');
subplot(2,1,2)
plot(t,0.5*(T2+dT2).*z(:,2),t,0.5*(T2-dT2).*z(:,4));
legend('left','right');
ylabel('power (W)');
xlabel('time (s)');
