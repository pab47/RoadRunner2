% format long
% close all
% clc
% clear all

function main(parameter)

parms = get_parms;
qL = 0;%angle should always be zero
uL = -0.1; %-0.5%mid-stance velocity 
qR = qL+0*(pi/parms.n); 
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

%change stiffness, change damping, change offset angle, angle made with the vertical, spokes
%parameter = 'alpha'; %parms.control.alpha
% parameter = 'stiffness'; %parms.k 
%parameter = 'spokes'; %parms.n
% parameter = 'com'; %parms.c
%parameter = 'mWheel'; %parms.m1

save_on = 1;

if (strcmp(parameter,'alpha'))
    alpha = 10:10:90;
    variable = alpha;
elseif (strcmp(parameter,'spokes'))
    %q2 = 90*(pi/180); 
%     uL = -5;
%     uR = uL;
    spokes = 8:2:20;
%    spokes = 4;
    variable = spokes;
elseif (strcmp(parameter,'stiffness'))
    k = 500:500:4000;
    variable = k;
elseif (strcmp(parameter,'com'))    
    c = 0.02:0.02:0.2;
    variable = c;
elseif (strcmp(parameter,'mWheel')) 
    mWheel = 2:0.5:6;
    variable = mWheel;
else
    error('Incorrect parameter value');
end

for i=1:length(variable)
    
    if (strcmp(parameter,'alpha'))
        q2 = variable(i)*(pi/180);
    elseif (strcmp(parameter,'spokes'))
        parms.n = variable(i);
    elseif (strcmp(parameter,'stiffness'))
        parms.k = variable(i);
    elseif (strcmp(parameter,'com'))    
        parms.c = variable(i);
    elseif (strcmp(parameter,'mWheel'))    
        parms.m1 = variable(i);
    else
        error('Incorrect parameter value');
    end
    variable(i)

    z0 = [qL uL qR uR q2 u2 l0 l0dot xmid0 ymid0 phi0 e_q2];% [angle rate];
    [z,t,steps,T2,dT2,error_flag] = forward_dynamics(z0,parms,total_time);

% disp('x    y     phi');
% disp([z(end,9) z(end,10) z(end,11)*(180/pi)]);
% %Dist_x = z(end,9)
% %Speed_x = z(end,9)/t(end)
disp(['Number of steps is ',num2str(steps)]);    


%size(z)
% if (error_flag ~= 0) %%0 no failure, 1 = ground penetration, 2 = ground reaction force < 0 
%     warning('simulation failed: change initial condition or torso angle');
% end
% 
% disp(['Number of steps is ',num2str(steps)]);    
%     disp('Animating...');
% 

% 

%save('sim_data.mat','t','z','T2','dT2');

% figure(1)
% subplot(2,1,1)
% plot(t,z(:,1),t,z(:,3));
% ylabel('absolute angles (rad)');
% legend('left','right');
% subplot(2,1,2)
% plot(t,z(:,2),t,z(:,4));
% ylabel('absolute angular rates (rad/s)');
% legend('left','right');
% xlabel('time (s)');

% figure(2)
% subplot(2,1,1);
% plot(t,z(:,5)*(180/pi));
% ylabel('torso angle (deg)');
% subplot(2,1,2);
% plot(t,z(:,6));
% ylabel('torso speed rad/s');
% xlabel('time (s)');


mean_torque = mean(T2);
mean_speed_left = mean(z(:,2));
mean_speed_right = mean(z(:,4));
mean_power_left = abs(mean((0.5*T2+dT2).*z(:,2)));
mean_power_right = abs(mean((0.5*T2-dT2).*z(:,4)));
robot_speed = abs((z(end,9) - z(1,9))/(t(end)-t(1)));
power_pi = 5;
power_teensy = 0.2;
power_motor = abs(mean_power_left+mean_power_right);
TCOT = (mean_power_left+mean_power_right+power_pi+power_teensy)/ ((parms.m1+parms.m2)*parms.g*robot_speed);
TCOT_motor = (mean_power_left+mean_power_right)/ ((parms.m1+parms.m2)*parms.g*robot_speed);
%disp('mean torque on torso = '); 
%disp(mean_torque);
%disp('mean speed = ')
%disp(['left = ', num2str(mean_speed_left),';',' right = ', num2str(mean_speed_right)]);
%disp('mean power = ')
%disp(['left = ', num2str(mean_power_left),';',' right = ', num2str(mean_power_right)]);
disp(['Mean Power = ',num2str(power_motor)]);
disp(['Mean Torque = ',num2str(mean_torque)]);
disp(['Speed = ',num2str(robot_speed)]);
disp(['TCOT = ',num2str(TCOT)]); 
disp(' ');
disp(' ');

power_all(i,1) = power_motor;
torque_all(i,1) = mean_torque;
robot_speed_all(i,1) = robot_speed;
TCOT_all(i,1) = TCOT;
TCOT_motor_all(i,1) = TCOT_motor;
end

if (save_on==1)
    if (strcmp(parameter,'alpha'))
        save('parm_alpha','alpha','power_all','torque_all','robot_speed_all','TCOT_all','TCOT_motor_all');
    elseif (strcmp(parameter,'spokes'))
        save('parm_spokes','spokes','power_all','torque_all','robot_speed_all','TCOT_all','TCOT_motor_all');
    elseif (strcmp(parameter,'stiffness'))
        save('parm_stiffness','k','power_all','torque_all','robot_speed_all','TCOT_all','TCOT_motor_all');
    elseif (strcmp(parameter,'com'))    
        save('parm_com','c','power_all','torque_all','robot_speed_all','TCOT_all','TCOT_motor_all');
    elseif (strcmp(parameter,'mWheel'))    
        save('parm_mWheel','mWheel','power_all','torque_all','robot_speed_all','TCOT_all','TCOT_motor_all');
    else
        error('Incorrect parameter value');
    end
end
% figure(3)
% subplot(2,1,1)
% plot(t,T2+dT2,t,T2-dT2);
% legend('left','right');
% ylabel('torque (Nm)');
% subplot(2,1,2)
% plot(t,0.5*(T2+dT2).*z(:,2),t,0.5*(T2-dT2).*z(:,4));
% legend('left','right');
% ylabel('power (W)');
% xlabel('time (s)');


% AZ = -62; EL = 26;
% %AZ = 0; EL = 0;
% %AZ = -90; EL = 90;
% %AZ = -44; EL = 16;
% view_angle = [AZ EL];
% 
% disp('Animating...');
% figure(4)
% animation3D(t,z,parms,fps,steps,view_angle);
% close(figure(4))
