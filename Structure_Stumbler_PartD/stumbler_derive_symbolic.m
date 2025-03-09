clear all; clc
%% Initialize symbolic variables
% Create symbolic variables for the angles, angular velocities, limb lengths
% and CoG locations. 
% 1) Angles
syms gamma1 alpha2 beta2 gamma2 gamma3 gamma4 real;
% 2) Angular velocities
syms gamma1dot alpha2dot beta2dot gamma2dot gamma3dot gamma4dot real;

% 3) Limb lengths
syms Lfoot Lshank Lthigh Lstance Lhip real;

% 4) Centers of gravity of limbs
syms cgFoot cgShank cgThigh cgStance real;


%% Define joint and mass locations
% Make sure that the required rotation matrices are saved as a function
% in the same folder as this file. (rotz, roty, rotx)
NCH = rotz(gamma1); %N is the reference frame of the surrounding, H of left leg/hat

TCH = roty(beta2)*rotx(alpha2)*rotz(gamma2); %T is the reference frame of the thigh
HCT = inv(TCH);
NCT = NCH * HCT;

LCT = rotz(gamma3);
TCL = inv(LCT);
NCL = NCT * TCL; %L is the reference frame with respect to the leg

FCL = rotz(gamma4);
LCF = inv(FCL);
NCF = NCL * LCF; %F is the reference frame with respect to the foot

%v_contactpoint_N = [0;0;0];
%v_m_HAT_H = [0;l_HAT_ver;com_HAT];
%v_m_HAT_N = N_C_H*v_m_HAT_H;
%v_righthip_H = [0;l_HAT_ver;l_HAT_hor];
%v_righthip_N = N_C_H*v_righthip_H;
%v_m_thigh_N = v_righthip_N + N_C_T*[0;-com_thigh;0];
%v_knee_N = v_righthip_N + N_C_T*[0;-l_thigh;0];
%v_m_leg_N = v_knee_N + N_C_L * [0;-com_leg;0];
%v_ankle_N = v_knee_N + N_C_L * [0;-l_leg;0];
%v_m_foot_N = v_ankle_N + N_C_F * [0;-com_foot;0];
%v_toe_N = v_ankle_N + N_C_F * [0;-l_foot;0];

Xlefthip    = NCH*[0; Lstance; 0];   %finish the locations yourself. How many should you define?
Xmhip       = Xlefthip + [0; 0; Lhip*0.5];
Xrighthip   = Xlefthip + [0;0;Lhip];
Xmthigh     = Xrighthip + NCT*[0; -cgThigh; 0];
Xknee       = Xrighthip + NCT*[0; -Lthigh; 0];
Xmshank     = Xknee + NCL*[0; -cgShank; 0];
Xankle      = Xknee + NCL*[0; -Lshank; 0];
Xmfoot      = Xankle + NCF*[cgFoot; 0; 0];
Xtoe        = Xankle + NCF*[Lfoot; 0 ; 0];

%% Define state vector and state derivative vector

q       = [gamma1; alpha2; beta2; gamma2; gamma3; gamma4];
qdot    = [gamma1dot; alpha2dot; beta2dot; gamma2dot; gamma3dot; gamma4dot];

%% Transformation function Ti, its derivative Ti_k and convective acceleration.
Ti      = [Xlefthip; Xmhip; Xrighthip; Xmthigh; Xknee; Xmshank; Xankle; Xmfoot;Xtoe];
Ti_k    = jacobian(Ti, q);
gconv   = jacobian(Ti_k*qdot, q)*qdot; 

%% Save symbolic derivation to script file.
% Use Diary function, save the symbolicly derived functions to file. 
% Save symbolic derivation to script file.
if exist('symb_Ti.m', 'file')
    ! del symb_Ti.m
end
diary symb_Ti.m
    disp('Ti = ['), disp(Ti), disp('];');
diary off

if exist('symb_Ti_k.m', 'file')
    ! del symb_Ti_k.m
end
diary symb_Ti_k.m
    disp('Ti_k = ['), disp(Ti_k), disp('];');
diary off

if exist('symb_gconv.m', 'file')
    ! del symb_gconv.m
end
diary symb_gconv.m
    disp('gconv = ['), disp(gconv), disp('];')
diary off