
%%%%%%%% Note that sim_data.mat comes from 3D simulations %%%%%%%%
clc
clear all
close all

load('sim_data.mat');

i=1;
figure(i); i=i+1;
plot(t,z(:,5),'r','Linewidth',3); hold on;


figure(i); i=i+1;
plot(t,T2,'r','Linewidth',3);  hold on;



figure(i); i=i+1;
plot(t,0.5*(z(:,2)+z(:,4)),'r','Linewidth',3);  hold on;



figure(i); i=i+1;
plot(t,(-0.5*(T2+dT2)-0.5*(T2-dT2)).*z(:,2),'r','Linewidth',2);  hold on;


disp('values from simulation data');
power_motor = abs(mean(0.5*(T2+dT2).*z(:,2))+ mean(0.5*(T2-dT2).*z(:,4)))
power_pi = 5;
power_teensy = 0.2;
power_total = power_motor + power_pi + power_teensy
mass = 6.9;
g = 9.81;
leg_length = 0.26;
mean_torque = mean(0.5*(T2+dT2)+0.5*(T2-dT2))
speed_rad_s = -mean(0.5*(z(:,2)+z(:,4)));
speed_m_s = leg_length*speed_rad_s
COT = power_total/ (mass*g*speed_m_s)
