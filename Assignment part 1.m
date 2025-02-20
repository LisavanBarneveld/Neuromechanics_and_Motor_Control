clear all; clc
%% Initialize symbolic variables
% Create symbolic variables for the angles, angular velocities, limb lengths
% and CoG locations. 
% 1) Angles

% 2) Angular velocities

% 3) Limb lengths

% 4) Centers of gravity of limbs


%% Define joint and mass locations
% Make sure that the required rotation matrices are saved as a function
% in the same folder as this file. (rotz, roty, rotx)

Xlefthip    =    %finish the locations yourself. How many should you define?
Xmhip       =
Xrighthip   =
Xmthigh     =
Xknee       =
Xmshank     =
Xankle      =
Xmfoot      =
Xtoe        =

%% Define state vector and state derivative vector

q       = 
qdot    = 

%% Transformation function Ti, its derivative Ti_k and convective acceleration.
Ti      = 
Ti_k    =
gconv   = 

%% Save symbolic derivation to script file.
% Use Diary function, save the symbolicly derived functions to file. 
