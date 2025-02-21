clear all; clc
%% Initialize symbolic variables
% Create symbolic variables for the angles, angular velocities, limb lengths
% and CoG locations. 

% 1) Angles

syms gamma1 alpha2 beta2 gamma2 gamma3 gamma4 real;
q=[gamma1;alpha2;beta2;gamma2;gamma3;gamma4];

gamma1 = 0; % temporary values!!!
alpha2 = 0;
beta2 = 0;
gamma2 = 0;
gamma3 = 0;
gamma4 = 90;

% 2) Angular velocities

syms gamma1dot alpha2dot beta2dot gamma2dot gamma3dot gamma4dot real;

% 3) Limb lengths

syms l_foot l_leg l_thigh l_HAT_ver l_HAT_hor real;
l_foot = 0.195;
l_leg = 0.435;
l_thigh = 0.410;
l_HAT_hor = 0.295;
l_HAT_ver = l_leg + l_thigh;

% 4) Centers of gravity of limbs

syms com_foot com_leg com_thigh com_HAT real;
com_foot = 0.098;
com_leg = 0.188;
com_thigh = 0.178;
com_HAT = 0.1475; %decision was made that com for the HAT should be in the middle of the vertical part of HAT

% 5) Masses

syms m_foot m_leg m_thigh m_HAT real;
m_foot = 1.16;
m_leg = 3.72;
m_thigh = 8.0;
m_HAT = 54.24;



%% Define joint and mass locations
% Make sure that the required rotation matrices are saved as a function
% in the same folder as this file. (rotz, roty, rotx)

%define rotation matrices
function rot_mat=rot_x(angle)
    rad = deg2rad(angle);
    rot_mat = [1,0,0; 0,cos(rad),sin(rad); 0, -sin(rad),cos(rad)];
end

function rot_mat=rot_y(angle)
    rad=deg2rad(angle);
    rot_mat=[cos(angle),0,-sin(angle);0,1,0;sin(angle),0,cos(angle)];
end

function rot_mat=rot_z(angle)
    rad=deg2rad(angle);
    rot_mat=[cos(angle),sin(angle),0;-sin(angle), cos(angle),0;0,0,1];
end

%define locations of centre of masses and other locations
N_C_H = rot_z(gamma1); %N is the reference frame of the surrounding, H of left leg/hat

T_C_H = rot_y(beta2)*rot_x(alpha2)*rot_z(gamma2); %T is the reference frame of the thigh
H_C_T = inv(T_C_H);
N_C_T = N_C_H * H_C_T;

L_C_T = rot_z(gamma3);
T_C_L = inv(L_C_T);
N_C_L = N_C_T * T_C_L; %L is the reference frame with respect to the leg

F_C_L = rot_z(gamma4);
L_C_F = inv(F_C_L);
N_C_F = N_C_L * L_C_F; %F is the reference frame with respect to the foot

v_contactpoint_N = [0;0;0];

v_m_HAT_H = [0;l_HAT_ver;com_HAT];
v_m_HAT_N = N_C_H*v_m_HAT_H;

v_righthip_H = [0;l_HAT_ver;l_HAT_hor];
v_righthip_N = N_C_H*v_righthip_H;

v_m_thigh_N = v_righthip_N + N_C_T*[0;-com_thigh;0];

v_knee_N = v_righthip_N + N_C_T*[0;-l_thigh;0];

v_m_leg_N = v_knee_N + N_C_L * [0;-com_leg;0];

v_ankle_N = v_knee_N + N_C_L * [0;-l_leg;0];

v_m_foot_N = v_ankle_N + N_C_F * [0;-com_foot;0];

v_toe_N = v_ankle_N + N_C_F * [0;-l_foot;0];


% plot the locations
locations = [v_contactpoint_N, v_m_HAT_N, v_righthip_N, v_m_thigh_N, v_knee_N, v_m_leg_N, v_ankle_N, v_m_foot_N, v_toe_N];

figure;
plot3(locations(1, :), locations(2, :), locations(3, :), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Add labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Plot of all defined locations');

grid on;

    
% Xlefthip    =    %finish the locations yourself. How many should you define?
% Xmhip       =
% Xrighthip   =
% Xmthigh     =
% Xknee       =
% Xmshank     =
% Xankle      =
% Xmfoot      =
% Xtoe        =
% 
% %% Define state vector and state derivative vector
% 
% q       = 
% qdot    = 
% 
% %% Transformation function Ti, its derivative Ti_k and convective acceleration.
% Ti      = 
% Ti_k    =
% gconv   = 
% 
% %% Save symbolic derivation to script file.
% % Use Diary function, save the symbolicly derived functions to file. 
