function parms = get_parms

%parms.fixed = 0; %1 for torso fixed
                 %anything else uses torso with dynamics
parms.g = 9.81;
parms.n = 10; %spokes
parms.gam = 0.0; %slope
parms.ignore_VelocityCondition = 0; %detect backward motion
parms.control.alpha = 50*(pi/180); %torso angle
parms.makeAVI = 0; %

%%%%%% https://www.mcmaster.com/9657k422 
%%% spring rate 15 lbs/in = 2627 N/m, max compression (delta x) = 2.5-0.87 = 1.63 in = 0.04 m 
parms.k = 2627/2; %springs are in parallel
parms.cv = 10^5; %10^5;

m_wheel = (2.32/6.51)*6.9;
m_torso = (4.194/6.51)*6.9;
leg_length = 0.26;
torso_com =   0.048*1.25;          %original = 0.048;
parms.b = 0.31/2; %half robot width
parms.offsetz_box = 3*2.54/100; %offset of the geometric center of box from the main shaft
parms.lbox = (5*2.54)/100;
parms.tbox = (1.5*2.54)/100;

%%%%%% static torque 
%m_torso*parms.g*torso_com*sin(50*pi/180)

% if (parms.fixed == 1)
%     parms.m = m_wheel; %1;
%     parms.M = m_torso;% 1;
%     parms.l = leg_length;% 1;
%     parms.L = torso_com; %0.5; %distance of torso from pin joint
%     
%     
%     %parms.ignore_VelocityCondition = 0; %detect backward motion
%     %parms.control.alpha = 45*(pi/180);%0.5401; %0.54006 %1.33;  %0.57 (%1.3961);
%     parms.disturb.height = 0.0*parms.l; 
%     parms.disturb.gam = 0;
% else
    parms.m1 = m_wheel;% 1;
    parms.m2 = m_torso; %1;
    parms.I1 = 0.0;
    parms.I2 = 0.0;
    parms.It = 0.052; %transverse inertia
    parms.l0 = leg_length; %1;
    parms.c = torso_com; %0.5;
    parms.w = 0;
    gearing = 4.5;
    kt = (2*gearing)/( (280*2*pi)/60);
    dI = 1.5;
 
    %parms.control.alpha = 45*(pi/180);%0.5401; %0.54006 %1.33;  %0.57 (%1.3961);
    parms.disturb.height = 0.0*parms.l0; 
    %parms.disturb.gam = 0;

    parms.control.V = 0;
    parms.control.kp_q2 = 400;
    parms.control.kp_u2 = parms.control.kp_q2*0.1;
    parms.control.kp_int = 10000;
    parms.control.q2 = parms.control.alpha; %torso angle %0.25;
    parms.control.u2 = 0;
    parms.control.kp_u1 = 0; 
    parms.control.u1 = 0;
    parms.control.dT2 = kt*dI;
    parms.control.udiff = 0;
    parms.control.kp_udiff = 0.1;
    %parms.control.T2 = 0;

%end

