%%% This file derives the saggital plane model of a rimless wheel with
%%% compliant spokes %%%%

%%% See supplement to the paper An differential-drive rimless wheel that 
%%% can move straight and turn: design, modeling, and control for more
%%% details

clc;
clear all

syms q1 q2 l l0 real  %angles, length 
syms q1_m q2_m l_m real %angles, length before collision
syms q1dot q2dot ldot real   %velocity 
syms q1dot_m q2dot_m ldot_m real %velocity before collision
syms q1ddot q2ddot lddot real %acceleration
syms g real % gravity
syms c w n real % com of torso (c,w), number of spokes
syms m1 m2 real  %masses 1 wheel, 2 torso
syms I1 I2 real %inertia 1 wheel, 2 torso
syms T2 F_r F_th real %torque due to torso and forces in the spring (radial and transverse)

i=[1 0 0];
j=[0 1 0];
k=[0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 Reference Frames                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

phi1 = q1+pi/2;
er1 = simplify(cos(phi1)*i+sin(phi1)*j);
eth1 = simplify(-sin(phi1)*i+cos(phi1)*j);

phi2 = q2-pi/2;
er2 = simplify(cos(phi2)*i+sin(phi2)*j);
eth2 = simplify(-sin(phi2)*i+cos(phi2)*j);

phiF = (q1+2*pi/n) + pi/2;
erF = simplify(cos(phiF)*i+sin(phiF)*j);
ethF = simplify(-sin(phiF)*i+cos(phiF)*j);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Position Vectors                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r_G1_G2 = c*er2;
r_P_G1  = l*er1;
r_P_G2 = r_P_G1 + r_G1_G2;

r_Q_G1  = l0*erF;
r_Q_G2  = r_Q_G1 + r_G1_G2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%        Angular Velocities and Accelerations       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

om1 = q1dot*k;
om2 = q2dot*k;
 
al1 = q1ddot*k;
al2 = q2ddot*k;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Constraints, Linear Velocities and Accelerations %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

v1 = ldot*er1 + l*q1dot*eth1;
v2 = v1 + c*q2dot*eth2;
a1 = (lddot-l*q1dot^2)*er1 + (2*ldot*q1dot+l*q1ddot)*eth1;
a2 = a1 -c*q2dot^2*er2 + c*q2ddot*eth2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Angular Momentum, (Before and After Heelstrike  % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

H_Pp = dot(...
            m1*cross(r_P_G1,v1) +  ...
            m2*cross(r_P_G2,v2) + ...
            I1*om1 + I2*om2, ...
            k);
H_Qn = subs(...
                dot(...
                   m1*cross(r_Q_G1,v1) +  ...
                   m2*cross(r_Q_G2,v2) +  ...
                   I1*om1 + I2*om2, ...
                   k)...
            ,{q1,q2,l,q1dot,q2dot,ldot},{q1_m,q2_m,l_m,q1dot_m,q2dot_m,ldot_m});
        

H_G1p = dot(...
           m2*cross(r_G1_G2,v2) + I2*om2  ...
           ,k);
H_G1n = subs(...
             dot(...
                m2*cross(r_G1_G2,v2) + I2*om2 ...
                ,k),...
             {q1,q2,l,q1dot,q2dot,ldot},{q1_m,q2_m,l_m,q1dot_m,q2dot_m,ldot_m});  %v_G2 is questionable.

LM_er1_p = simplify(dot(m1*v1 + m2*v2,er1));
LM_er1_n = subs(LM_er1_p,{q1,q2,l,q1dot,q2dot,ldot},{q1_m,q2_m,l_m,q1dot_m,q2dot_m,ldot_m});


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   EOM:[M-Hdot=0](Single Stance)  % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

LMB_er1 = simplify(dot(F_r*er1 + F_th*eth1  - m1*g*j -m2*g*j -m1*a1 - m2*a2,er1));
AMB1 = cross(r_P_G1,m1*g*(-j)) + cross(r_P_G2,m2*g*(-j)) ...
        -cross(r_P_G1,m1*a1) -cross(r_P_G2,m2*a2) -I1*al1 - I2*al2;    
AMB2 = T2*k + cross(r_G1_G2,m2*g*(-j)) - cross(r_G1_G2,m2*a2) - I2*al2;


index = 3;
RHSs1 = -subs(AMB1(index),[q1ddot q2ddot lddot],[0 0 0]); 
Ms11  =  subs(AMB1(index),[q1ddot q2ddot lddot],[1 0 0]) + RHSs1;
Ms12  =  subs(AMB1(index),[q1ddot q2ddot lddot],[0 1 0]) + RHSs1;
Ms13  =  subs(AMB1(index),[q1ddot q2ddot lddot],[0 0 1]) + RHSs1;

index = 3;
RHSs2 = -subs(AMB2(index),[q1ddot q2ddot lddot],[0 0 0]); 
Ms21  =  subs(AMB2(index),[q1ddot q2ddot lddot],[1 0 0]) + RHSs2;
Ms22  =  subs(AMB2(index),[q1ddot q2ddot lddot],[0 1 0]) + RHSs2;
Ms23  =  subs(AMB2(index),[q1ddot q2ddot lddot],[0 0 1]) + RHSs2;

RHSs3 = -subs(LMB_er1,[q1ddot q2ddot lddot],[0 0 0]); 
Ms31  =  subs(LMB_er1,[q1ddot q2ddot lddot],[1 0 0]) + RHSs3;
Ms32  =  subs(LMB_er1,[q1ddot q2ddot lddot],[0 1 0]) + RHSs3;
Ms33  =  subs(LMB_er1,[q1ddot q2ddot lddot],[0 0 1]) + RHSs3;

disp(' ');
disp('Equation of single stance');
disp(' ');

disp(['M11 = ', char(simplify(Ms11)),';']);
disp(['M21 = ', char(simplify(Ms21)),';']);
disp(['M31 = ', char(simplify(Ms31)),';']);
disp(['M12 = ', char(simplify(Ms12)),';']);
disp(['M22 = ', char(simplify(Ms22)),';']);
disp(['M32 = ', char(simplify(Ms32)),';']);
disp(['M13 = ', char(simplify(Ms13)),';']);
disp(['M23 = ', char(simplify(Ms23)),';']);
disp(['M33 = ', char(simplify(Ms33)),';']);
disp(['RHS1 = ', char(simplify(RHSs1)),';']);
disp(['RHS2 = ', char(simplify(RHSs2)),';']);
disp(['RHS3 = ', char(simplify(RHSs3)),';']);

% disp('single stance in latex');
% latex(simplify(Ms11))
% latex(simplify(Ms12))
% latex(simplify(Ms13))
% latex(simplify(Ms21))
% latex(simplify(Ms22))
% latex(simplify(Ms23))
% latex(simplify(Ms31))
% latex(simplify(Ms32))
% latex(simplify(Ms33))
% latex(simplify(RHSs1))
% latex(simplify(RHSs2))
% latex(simplify(RHSs3))

disp(' ')
disp(' ')

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Heelstrike:[H+ = H-](Single Stance) %  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

RHShs1 = H_G1n;
RHShs2 = H_Qn;
RHShs3 = LM_er1_n;

Mhs11  =  subs(H_G1p,[q1dot q2dot ldot],[1 0 0]);
Mhs12  =  subs(H_G1p,[q1dot q2dot ldot],[0 1 0]);
Mhs13  =  subs(H_G1p,[q1dot q2dot ldot],[0 0 1]);

Mhs21  =  subs(H_Pp,[q1dot q2dot ldot],[1 0 0]);
Mhs22  =  subs(H_Pp,[q1dot q2dot ldot],[0 1 0]);
Mhs23  =  subs(H_Pp,[q1dot q2dot ldot],[0 0 1]);

Mhs31  =  subs(LM_er1_p,[q1dot q2dot ldot],[1 0 0]);
Mhs32  =  subs(LM_er1_p,[q1dot q2dot ldot],[0 1 0]);
Mhs33  =  subs(LM_er1_p,[q1dot q2dot ldot],[0 0 1]);


disp('Equation of heelstrike');
disp(' ');

disp(['M11 = ', char(simplify(Mhs11)),';']);
disp(['M21 = ', char(simplify(Mhs21)),';']);
disp(['M31 = ', char(simplify(Mhs31)),';']);
disp(['M12 = ', char(simplify(Mhs12)),';']);
disp(['M22 = ', char(simplify(Mhs22)),';']);
disp(['M32 = ', char(simplify(Mhs32)),';']);
disp(['M13 = ', char(simplify(Mhs13)),';']);
disp(['M23 = ', char(simplify(Mhs23)),';']);
disp(['M33 = ', char(simplify(Mhs33)),';']);
disp(['RHS1 = ', char(simplify(RHShs1)),';']);
disp(['RHS2 = ', char(simplify(RHShs2)),';']);
disp(['RHS3 = ', char(simplify(RHShs3)),';']);

% disp('heelstrike in latex');
% latex(simplify(Mhs11))
% latex(simplify(Mhs12))
% latex(simplify(Mhs13))
% latex(simplify(Mhs21))
% latex(simplify(Mhs22))
% latex(simplify(Mhs23))
% latex(simplify(Mhs31))
% latex(simplify(Mhs32))
% latex(simplify(Mhs33))
% latex(simplify(RHShs1))
% latex(simplify(RHShs2))
% latex(simplify(RHShs3))

disp(' ');
disp('End of derivation');