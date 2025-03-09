function [sys,x0,str,ts] = sfun_stumbler(t,x,u,flag)
global model
%% Define model masses, lengths, stiffness, damping and gravity
% Find reasonable values for the following variables and store them in 
% the model structure, defined globally above. 
% example: model.mstance = <your value here> %[kg]; 
% (hint; consider the dimensions (single value/vectors) of the variables)

% Masses
massbody = 80; %kg
model.mstance   = (0.0465+0.1+0.0145)*massbody;   % mass stance leg [kg]
model.mhip      = 0.678*massbody;   % mass trunk on hip [kg
model.mthigh    = 0.1*massbody;   % mass thigh swing leg [kg]
model.mshank    = 0.0465*massbody;   % mass shank swing leg [kg]
model.mfoot     = 0.0145*massbody;   % mass foot swing leg [kg]

% Lengths
model.Lstance   = 0.845;% [m]
model.Lhip      = 0.295;% [m]
model.Lthigh    = 0.410;% [m]
model.Lshank    = 0.435;% [m]
model.Lfoot     = 0.195;% [m]

% Centres of gravity with respect to proximal joint
model.cgStance =  0.447*0.845;% [m]
model.cgThigh  =  0.433*0.410;% [m]
model.cgShank  =  0.433*0.435;% [m]
model.cgFoot   =  0.5*0.195;% [m]

% Joint stiffness
%gamma1, alpha2, beta2, gamma2, gamma3, gamma4
model.kjoint    = [6; 6; 10; 6; 1000; 6];

% Joint damping
model.bjoint    = [6; 6; 6; 6; 100; 6];

% Gravity
model.g         = 9.81; % [Nm/s^2]

% % Part B: brick dimensions
xbrick          = 0;
model.bx1       = xbrick;
model.bx2       = xbrick + 0.06;
model.by1       = 0;
model.by2       = 0.10;
model.bz1       = 0.17;
model.bz2       = 0.45;
model.bbrick    = 500;
model.kbrick    = 5000;

%% 
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,model);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes

% Init sizes
sizes = simsizes;
sizes.NumContStates  = 12;     % Two times the number of generalized coordinates
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;       % All joint angles
sizes.NumInputs      = 6;       % All joint torques
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;       % At least one sample time is needed
sys = simsizes(sizes);

% Initial angles [rad]
% Define what the initial values of the angles are
gamma1  = 25*pi/180;
alpha2  = 0;
beta2   = 0;
gamma2  = 50*pi/180;
gamma3  = 0;
gamma4  = -25*pi/180;

% Initial angular velocities [rad/s]
% Define what the initial values of the angular velocities are
gamma1dot   = -1.8;
alpha2dot   = 0;
beta2dot    = 0;
gamma2dot   = 0;
gamma3dot   = 0;
gamma4dot   = 0;

% States and state derivatives
q       = [gamma1; alpha2; beta2; gamma2; gamma3; gamma4];
qdot    = [gamma1dot; alpha2dot; beta2dot; gamma2dot; gamma3dot; gamma4dot];

x0  = [q; qdot];


% str is always an empty matrix
str = [];


% initialize the array of sample times
ts  = [0 0];

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,model)
%% Get model parameters from model structure 
% Masses
mstance     = model.mstance;     % mass stance leg [kg]
mhip        = model.mhip;        % mass trunk on hip [kg
mthigh      = model.mthigh;      % mass thigh swing leg [kg]
mshank      = model.mshank;      % mass shank swing leg [kg]
mfoot       = model.mfoot;      % mass foot swing leg [kg]

% Lengths
Lstance     = model.Lstance;    % [m]
Lhip        = model.Lhip;       % [m]
Lthigh      = model.Lthigh;     % [m]
Lshank      = model.Lshank;     % [m]
Lfoot       = model.Lfoot;      % [m]

% Centres of gravity with respect to proximal joint
cgStance    = model.cgStance;   % [m]
cgThigh     = model.cgThigh;    % [m]
cgShank     = model.cgShank;    % [m]
cgFoot      = model.cgFoot;     % [m]

% Gravity
g           = model.g;

%% Determine mass matrix
mass = diag([0, 0, 0, mhip, mhip, mhip, 0, 0, 0, mthigh, mthigh,mthigh, 0, 0, 0, mshank, mshank, mshank, 0, 0, 0, mfoot, mfoot, mfoot, 0, 0, 0]);
    
%% Get state variables from state vector x
q           = x(1:6);
qdot        = x(7:12);
gamma1      = q(1);
alpha2      = q(2);
beta2       = q(3);
gamma2      = q(4);
gamma3      = q(5);
gamma4      = q(6);
gamma1dot   = qdot(1);
alpha2dot   = qdot(2);
beta2dot    = qdot(3);
gamma2dot   = qdot(4);
gamma3dot   = qdot(5);
gamma4dot   = qdot(6);


%% Add dynamics stiffness and damping to prevent overstretching joints
% First get the stored values of passive stiffness and damping from 
% the model structure

% Passive Stiffness
kjoint      = model.kjoint;

% Passive Damping
bjoint      = model.bjoint;

% Active stiffness and damping
% Create if statement, which prevents the knee to overstretch by adapting
% the stiffness and damping
if gamma3 > 0 % Knee overstretched
    kjoint(5) = 10;
    bjoint(5) = 1;
end

%% Obtain transformation vector Ti and its derivatives Ti_k and Ti_km
symb_Ti;
symb_Ti_k;

%% Detect collision with brick
% Implement the collision with the brick and calculate the resulting force.
toe_pos = Ti(25:27);  % Toe position
toe_vel = Ti_k(25:27,:) * qdot;  % Toe velocity

% Initialize force
Fbrick = [0; 0; 0];  

% Debugging: Print toe position
disp(['Time: ', num2str(t)]);
disp(['Toe Position: ', num2str(toe_pos')]);
disp(['Brick Bounds: X(', num2str(model.bx1), ',', num2str(model.bx2), ...
       ') Y(', num2str(model.by1), ',', num2str(model.by2), ...
       ') Z(', num2str(model.bz1), ',', num2str(model.bz2), ')']);

% Check if the foot is inside the brick
if (toe_pos(1) >= model.bx1 && toe_pos(1) <= model.bx2) && ...
   (toe_pos(2) >= model.by1 && toe_pos(2) <= model.by2) && ...
   (toe_pos(3) >= model.bz1 && toe_pos(3) <= model.bz2)
    
    disp('🚨 Collision Detected! Applying force.');

    % Compute penetration depth
    penetration_x = model.bx2 - toe_pos(1);
    penetration_y = model.by2 - toe_pos(2);
    penetration_z = model.bz2 - toe_pos(3);
    
    % Apply Hooke's law force
    Fbrick(1) = -model.kbrick * penetration_x - model.bbrick * toe_vel(1);
    Fbrick(2) = -model.kbrick * penetration_y - model.bbrick * toe_vel(2);
    Fbrick(3) = -model.kbrick * penetration_z - model.bbrick * toe_vel(3);
else
    disp('No collision detected.');
end



%% Determine state derivatives
% Calculate reduced mass matrix Mred 
Mred = Ti_k.' * mass * Ti_k;

% Calculate convective acceleration 
symb_gconv;

% Reduced force vector 
% Apply force to the system
f = mass * [0, -g, 0, 0, -g, 0, 0, -g, 0, 0, -g, 0, 0, -g, 0, 0, -g, 0, 0, -g, 0,Fbrick(1), Fbrick(2), Fbrick(3), 0, 0, 0 ].';
Fred = Ti_k.' * (f - mass * gconv) - (bjoint .* qdot) - (kjoint .* q) + u;
disp('Force Vector (Fred) after applying Fbrick:');
disp(Fred);
% Calcultate derivatives qddot 
qddot = Mred \ Fred;

sys = [qdot ; qddot];


global torque_log time_log last_logged_time

% Initialize storage variables if empty
if isempty(torque_log)
    torque_log = [];
    time_log = [];
    last_logged_time = -1; % Stores the last logged time
end

% Only store torques if this time step is new
if t > last_logged_time
    torque_log = [torque_log; u'];  % Store torques
    time_log = [time_log; t];        % Store time
    last_logged_time = t;            % Update last logged time
end




% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)
% Empty matrix
sys = [];
% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

% Create output
sys = x(1:6);

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
