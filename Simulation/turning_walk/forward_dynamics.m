%===================================================================
function [z,t,steps,T2_all,dT2_all,error_flag]=forward_dynamics(z0,parms,total_time)
%===================================================================
error_flag = 0;
if nargin<2
    error('need more inputs to onestep');
end

dt = 0.01;
t0 = 0;
t = t0;
z = z0;

%%%%%%%% initial state z0 = [x0 x0dot y0 y0dot q2 q2dot l ldot xmid0 ymid0 phi0]
% x0 = 0; x0dot = 0;
% y0 = parms.l; y0dot = parms.l*

options=odeset('abstol',1e-9,'reltol',1e-9,'Events',@collision);
steps = 0;

while (t0<total_time)
    tspan = linspace(0,dt,4);
    [tout, zout,te,ze,ie] = ode113(@single_stance,tspan,z0,options,parms);

    
%     disp(' iter ')
%     disp([num2str(length(T2_all)),'; ',num2str(length(t))]);
    
%     T2_all = [T2_all; T2_temp(2:end)];
%     dT2_all = [dT2_all; dT2_temp(2:end)];
    
    
    t = [t; t0+tout(2:end)];
    z = [z; zout(2:end,:)];
    t0 = t0+tout(end);
    z0 = zout(end,:);
    if (~isempty(ie))
         %ie
        steps = steps+length(ie);
%         tout
%         ie
%         te
%        z0
        zplus = heelstrike(z0,parms,ie);
        z0 = zplus;
        
%         if (z0(9)<-0.5) 
%             break;
%         end
        %ze
        %break;
     end
end

for i=1:length(t)
    [zdot,T2,dT2]=single_stance(t(i),z(i,:),parms);
    T2_all(i,1) = T2;
    dT2_all(i,1) = dT2;
end
T2_all(1,1) = T2_all(2,1);
dT2_all(1,1) = dT2_all(2,1);


%===================================================================
function [zdot,T2,dT2]=single_stance(t,z,parms)  
%===================================================================

qL = z(1); uL = z(2);
qR = z(3); uR = z(4);
q2 = z(5); q2dot = z(6);
l = z(7); ldot = z(8);
e_q2 = z(12);

q1 = 0.5*(qL+qR);
q1dot = 0.5*(uL+uR);
qdiff = 0.5*(qL-qR);
udiff = 0.5*(uL-uR);

%phiLdot = ul; phiRdot = ur;
phi = z(11);

T2_openloopcontrol = 0;% parms.control.V - u1; %open loop, motor model is V = T + w
T2_torsopositioncontrol = -parms.control.kp_q2*(q2-parms.control.q2)...
                          -parms.control.kp_u2*(q2dot-parms.control.u2)...
                          -parms.control.kp_int*e_q2; %position control
T2_wheelspeedcontrol = 0;%-parms.control.kp_u1*(u1-parms.control.u1);   %speed control
T2 = T2_openloopcontrol + T2_torsopositioncontrol + T2_wheelspeedcontrol;
%dT2 = parms.control.dT2;

m1 = parms.m1; m2 = parms.m2;
c = parms.c; l0 = parms.l0; g = parms.g; k = parms.k; cv = parms.cv;
It = parms.It; %w = parms.w;  
b = parms.b;
I1 = parms.I1; I2 = parms.I2;

% M11 = I1 + l^2*m1 + l^2*m2 - c*l*m2*cos(q1 - q2) - l*m2*w*sin(q1 - q2);
% M21 = -l*m2*(c*cos(q1 - q2) + w*sin(q1 - q2));
% M12 = I2 + c^2*m2 + m2*w^2 - c*l*m2*cos(q1 - q2) - l*m2*w*sin(q1 - q2);
% M22 = I2 + c^2*m2 + m2*w^2;
% RHS1 = c*g*m2*sin(gam - q2) - g*m2*w*cos(gam - q2) - g*l*m1*sin(gam - q1) - g*l*m2*sin(gam - q1) - c*l*m2*u1^2*sin(q1 - q2) + c*l*m2*u2^2*sin(q1 - q2) + l*m2*u1^2*w*cos(q1 - q2) - l*m2*u2^2*w*cos(q1 - q2);
% RHS2 = T2 + c*g*m2*sin(gam - q2) - g*m2*w*cos(gam - q2) - c*l*m2*u1^2*sin(q1 - q2) + l*m2*u1^2*w*cos(q1 - q2);
%  

% %%%%% 9/28/2018
% M11 = I1 + l^2*m1 + l^2*m2 - c*l*m2*cos(q1 - q2) - l*m2*w*sin(q1 - q2);
% M21 = -l*m2*(c*cos(q1 - q2) + w*sin(q1 - q2));
% M12 = I2 + c^2*m2 + m2*w^2 - c*l*m2*cos(q1 - q2) - l*m2*w*sin(q1 - q2);
% M22 = I2 + c^2*m2 + m2*w^2;
% RHS1 = c*g*m2*sin(gam - q2) - g*m2*w*cos(gam - q2) - g*l*m1*sin(gam - q1) - g*l*m2*sin(gam - q1) - c*l*m2*u1^2*sin(q1 - q2) + c*l*m2*u2^2*sin(q1 - q2) + l*m2*u1^2*w*cos(q1 - q2) - l*m2*u2^2*w*cos(q1 - q2);
% RHS2 = T2 + c*g*m2*sin(gam - q2) - g*m2*w*cos(gam - q2) - c*l*m2*u1^2*sin(q1 - q2) + l*m2*u1^2*w*cos(q1 - q2);
%  
% M = [M11 M12; M21 M22];
% RHS = [RHS1; RHS2];
% X = M\RHS;
% u1dot = X(1,1);
% u2dot = X(2,1);

% %%%%%%% Jan 22, 2019
% F_r = -k*(l-l0)-cv*ldot;
% M11 = -l*(l*m1 + l*m2 - c*m2*cos(q1 - q2));
% M21 = c*l*m2*cos(q1 - q2);
% M31 = 0;
% M12 = -c*m2*(c - l*cos(q1 - q2));
% M22 = -c^2*m2;
% M32 = c*m2*sin(q1 - q2);
% M13 = c*m2*sin(q1 - q2);
% M23 = c*m2*sin(q1 - q2);
% M33 = - m1 - m2;
% RHS1 = 2*l*ldot*m1*q1dot + 2*l*ldot*m2*q1dot + c*g*m2*sin(q2) - g*l*m1*sin(q1) - g*l*m2*sin(q1) - 2*c*ldot*m2*q1dot*cos(q1 - q2) + c*l*m2*q1dot^2*sin(q1 - q2) - c*l*m2*q2dot^2*sin(q1 - q2);
% RHS2 = c*g*m2*sin(q2) - T2 - 2*c*ldot*m2*q1dot*cos(q1 - q2) + c*l*m2*q1dot^2*sin(q1 - q2);
% RHS3 = g*m1*cos(q1) - F_r + g*m2*cos(q1) - l*m1*q1dot^2 - l*m2*q1dot^2 + c*m2*q2dot^2*cos(q1 - q2);

%%%%%%% Jan 25, 2019 (with inertia)
F_r = -k*(l-l0)-cv*ldot;
M11 = c*l*m2*cos(q1 - q2) - l^2*m1 - l^2*m2 - I1;
M21 = c*l*m2*cos(q1 - q2);
M31 = 0;
M12 = c*l*m2*cos(q1 - q2) - c^2*m2 - I2;
M22 = - I2 - c^2*m2;
M32 = c*m2*sin(q1 - q2);
M13 = c*m2*sin(q1 - q2);
M23 = c*m2*sin(q1 - q2);
M33 = - m1 - m2;
RHS1 = 2*l*ldot*m1*q1dot + 2*l*ldot*m2*q1dot + c*g*m2*sin(q2) - g*l*m1*sin(q1) - g*l*m2*sin(q1) - 2*c*ldot*m2*q1dot*cos(q1 - q2) + c*l*m2*q1dot^2*sin(q1 - q2) - c*l*m2*q2dot^2*sin(q1 - q2);
RHS2 = c*g*m2*sin(q2) - T2 - 2*c*ldot*m2*q1dot*cos(q1 - q2) + c*l*m2*q1dot^2*sin(q1 - q2);
RHS3 = g*m1*cos(q1) - F_r + g*m2*cos(q1) - l*m1*q1dot^2 - l*m2*q1dot^2 + c*m2*q2dot^2*cos(q1 - q2);

M = [M11 M12 M13; M21 M22 M23; M31 M32 M33];
RHS = [RHS1; RHS2; RHS3];
X = M\RHS;

%zdot = [q1dot X(1,1) q2dot X(2,1) ldot X(3,1)]';
q1ddot = X(1,1);
q2ddot = X(2,1);
lddot  = X(3,1);

%dT2 = -0.1*(udiff-1);
dT2 = parms.control.kp_udiff*(udiff-parms.control.udiff)+parms.control.dT2;
udot_diff = dT2/It;
%(uLd + uRd)*0.5 = u1dot
%(uLd - uRd)*0.5 = udot_diff
uLd = q1ddot + udot_diff;
uRd = q1ddot - udot_diff;

R = [cos(phi) -sin(phi) 0; ...
     sin(phi) cos(phi) 0; ...
     0 0 1];
v = [0.5*l*(uL + uR); ...
     0; ...
     0.5*(l/b)*(uR - uL)];
zdot = R*v;
xdot = zdot(1);
ydot = zdot(2);
phidot = zdot(3);

zdot = [uL uLd uR uRd q2dot q2ddot ldot lddot xdot ydot phidot (q2-parms.control.q2)]';
%zdot = [u1 X(1,1) u2 X(2,1)]';

%===================================================================
function zplus=heelstrike(zminus,parms,events)      
%===================================================================

rL = zminus(1);   vL = zminus(2);
rR = zminus(3);   vR = zminus(4);
q2_m = zminus(5); q2dot_m = zminus(6);
l_m =  zminus(7); ldot_m =  zminus(8);

m1 = parms.m1; m2 = parms.m2;
c = parms.c; l0 = parms.l0; n = parms.n;
I1 = parms.I1; I2 = parms.I2;

v_diff = 0.5*(vL - vR);
u_diff = v_diff; %maintains the difference

if (length(events)==1)
    if (events==1) %left collision
        q1_m = rL;
        q1dot_m = vL;
        
        q1 = (q1_m + 2*pi/n);
        qL = -rL; 
        
        qR = rR; %unchanged
        %uR = vR; %unchanged 
        
        q2 = q2_m; %unchanged

    elseif (events==2) %right collision
        q1_m = rR;
        q1dot_m = vR; 
        
        q1 = (q1_m + 2*pi/n);
        qR = -rR; 
        
        qL = rL;   %unchanged
        %uL = vL; %unchanged
         
        q2 = q2_m; %unchanged

    else
        error('event is incorrect');
    end
end

if (length(events)==2)
    qL = -rL;
    qR = -rR;
    q1_m = 0.5*(rL+rR);
    q1 = (q1_m + 2*pi/n);
    q2 = q2_m; %unchanged
    q1dot_m = 0.5*(vL+vR);
    
end


%%%%%% Jan 25, 2019 (with inertia)
l = l0; %added manually
M11 = -c*l*m2*cos(q1 - q2);
M21 = I1 + l^2*m1 + l^2*m2 - c*l*m2*cos(q1 - q2);
M31 = 0;
M12 = I2 + c^2*m2;
M22 = I2 + c^2*m2 - c*l*m2*cos(q1 - q2);
M32 = -c*m2*sin(q1 - q2);
M13 = -c*m2*sin(q1 - q2);
M23 = -c*m2*sin(q1 - q2);
M33 = m1 + m2;
RHS1 = I2*q2dot_m - m2*(c*cos(q2_m)*(ldot_m*sin(q1_m) - c*q2dot_m*cos(q2_m) + l_m*q1dot_m*cos(q1_m)) - c*sin(q2_m)*(ldot_m*cos(q1_m) + c*q2dot_m*sin(q2_m) - l_m*q1dot_m*sin(q1_m)));
RHS2 = I1*q1dot_m + I2*q2dot_m - m2*((l0*sin(q1_m + (2*pi)/n) - c*sin(q2_m))*(ldot_m*cos(q1_m) + c*q2dot_m*sin(q2_m) - l_m*q1dot_m*sin(q1_m)) - (l0*cos(q1_m + (2*pi)/n) - c*cos(q2_m))*(ldot_m*sin(q1_m) - c*q2dot_m*cos(q2_m) + l_m*q1dot_m*cos(q1_m))) + m1*(l0*cos(q1_m + (2*pi)/n)*(ldot_m*sin(q1_m) + l_m*q1dot_m*cos(q1_m)) - l0*sin(q1_m + (2*pi)/n)*(ldot_m*cos(q1_m) - l_m*q1dot_m*sin(q1_m)));
RHS3 = ldot_m*m1 + ldot_m*m2 - c*m2*q2dot_m*sin(q1_m - q2_m);
M = [M11 M12 M13; M21 M22 M23; M31 M32 M33];
RHS = [RHS1; RHS2; RHS3];
X = M\RHS;

% if (~isempty(events))
%     events
% end

if (length(events)==1)
    if (events==1) %left collision
        %qL = -rL; 
        %qR = rR;
        %u1 = 0.5*(uL+uR);
        %u_diff = 0.5*(uL - uR);
         uL = X(1,1);    
         uR = uL-u_diff;
    elseif (events==2) %right 
%         qL = rL;
%         qR = -rR;  
         uR = X(1,1);
         uL = uR+u_diff;
    else
        error('event is incorrect');
    end
end

if (length(events)==2)
    u1 = X(1,1);   
    uL = u1+u_diff;
    uR = u1-u_diff;
%     qL = -rL;
%     qR = -rR;
end

zplus = [qL uL qR uR q2 X(2,1) l0 X(3,1) zminus(9:12)];


% %===================================================================
% function [h, isterminal,direction]=midstance(t,x,parms)
% %===================================================================
% 
% %n = parms.n;
% q = x(1);
% 
% h = q;
% isterminal=1; %= 1 if the integration is to terminate at a zero of this event function, otherwise, 0.
% direction = -1; %= 0 if all zeros are to be located (the default), +1 if only zeros where the event function is increasing, and -1 if only zeros where the event function is decreasing.
% 


%===================================================================
function [h, isterminal,direction]=collision(t,x,parms)
%===================================================================

n = parms.n;
qL = x(1);
qR = x(3);
l  = x(7);

hL = l*cos(qL) - parms.l0*cos(qL+(2*pi/n));%+parms.disturb.height;
hR = l*cos(qR) - parms.l0*cos(qR+(2*pi/n));%+parms.disturb.height;
%h = [hL, hR];
h(1) = hL;
h(2) = hR;

%h = parms.l*cos(q) - parms.l*cos(pi/n)+parms.disturb.height; 
%h = q+pi/n;
isterminal(1:2)=1; %= 1 if the integration is to terminate at a zero of this event function, otherwise, 0.
direction(1:2) = -1; %= 0 if all zeros are to be located (the default), +1 if only zeros where the event function is increasing, and -1 if only zeros where the event function is decreasing.



% %===================================================================
% function flag=check_failure(z,parms)
% %===================================================================
% %q = z(:,1); %store angle
% m1 = parms.m1; m2 = parms.m2; %I1 = parms.I1; I2 = parms.I2; n = parms.n;
% l = parms.l; c = parms.c; w = parms.w; g = parms.g; gam = parms.gam;
% 
% flag = 0;
% 
% %%%%%%%% check ground penetration %%%%%%%
% qmin = min(z(:,1));
% qmax = max(z(:,1));
% if (qmin < (-pi/parms.n)-0.1 || qmax > (pi/parms.n)+0.1)
%     warning('robot walk failure: ground penetration');
%     flag = 1;
% end
% 
% %%%%%%%% check reaction forces %%%%%%%
% Fy = zeros(length(z),1);
% for i=1:length(Fy)
%     t = 0;    
%     q1 = z(i,1);
%     u1 = z(i,2);
%     q2 = z(i,3);
%     u2 = z(i,4);
%     zdot=single_stance(t,[q1 u1 q2 u2],parms);
%     ud1 = zdot(2);
%     ud2 = zdot(4);
%     %Fy(i,1) = cos(gam)*(m2*(ud2*(c*sin(q2) + w*cos(q2)) + u2^2*(c*cos(q2) - w*sin(q2)) - l*ud1*sin(q1) - l*u1^2*cos(q1)) - m1*(l*ud1*sin(q1) + l*u1^2*cos(q1))) - sin(gam)*(m2*(ud2*(c*cos(q2) - w*sin(q2)) - u2^2*(c*sin(q2) + w*cos(q2)) - l*ud1*cos(q1) + l*u1^2*sin(q1)) - m1*(l*ud1*cos(q1) - l*u1^2*sin(q1))) + g*(m1 + m2);
%     Fy(i,1) = cos(gam)*(m2*(ud2*(c*sin(q2) + w*cos(q2)) + u2^2*(c*cos(q2) - w*sin(q2)) - l*ud1*sin(q1) - l*u1^2*cos(q1)) - m1*(l*ud1*sin(q1) + l*u1^2*cos(q1))) - sin(gam)*(m2*(ud2*(c*cos(q2) - w*sin(q2)) - u2^2*(c*sin(q2) + w*cos(q2)) - l*ud1*cos(q1) + l*u1^2*sin(q1)) - m1*(l*ud1*cos(q1) - l*u1^2*sin(q1))) + g*(m1 + m2);
% 
%     %Fy(i,1) = (parms.M + parms.m)*(parms.g - u^2*cos(parms.gam - q) + ud*sin(parms.gam - q));
% end
% 
% Fymin = min(Fy);
% if (Fymin<0)
%      warning('robot walk failure: vertical reaction force is zero');
%      flag = 2;
% end
% 
% q2_all = z(:,3);
% index1 = isempty(find(q2_all > pi/2,1));
% index2 = isempty(find(q2_all < -pi/2,1));
% if (~index1 || ~index2)
%     warning('robot walk failure imminent: torso beyond horizontal');
%      flag = 3;
% end