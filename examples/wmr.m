%------------------------------------------%

% Wheeled Mobile Robot (WRM) - Davide De Lazzari

%------------------------------------------%


%% Dimensions

nx=5;  % No. of differential states
nu=2;  % No. of controls
nz=0;  % No. of algebraic states
ny=7; % No. of outputs
nyN=5; % No. of outputs at the terminal point
np=0; % No. of model parameters
nc=0; % No. of general constraints
ncN=0; % No. of general constraints at the terminal point
nbx = 2; % No. of bounds on states
nbu = 2; % No. of bounds on controls

% state and control bounds
nbx_idx = [4 5]; % indexes of states which are bounded
nbu_idx = [1 2]; % indexes of controls which are bounded

%% create variables

import casadi.*

states   = SX.sym('states',nx,1);   % differential states
controls = SX.sym('controls',nu,1); % control input
alg      = SX.sym('alg',nz,1);      % algebraic states
params   = SX.sym('paras',np,1);    % parameters
refs     = SX.sym('refs',ny,1);     % references of the first N stages
refN     = SX.sym('refs',nyN,1);    % reference of the last stage
Q        = SX.sym('Q',ny,1);        % weighting matrix of the first N stages
QN       = SX.sym('QN',nyN,1);      % weighting matrix of the last stage
aux      = SX.sym('aux',ny,1);      % auxilary variable
auxN     = SX.sym('auxN',nyN,1);    % auxilary variable

%% Dynamics

z = states(1); % z axis = horizontal
y = states(2); % y axis = vertical
th = states(3); % anticlockwise angle wrt z axis
vl = states(4); % left wheel vel
vr = states(5); % right wheel vel

al = controls(1); % left wheel acc
ar = controls(2); % right wheel acc

d = 0.6; %approximate distance between wheels
v = (vr+vl)/2;
w = (vr-vl)/d;

% explicit ODE RHS
x_dot=[v*sin(th);v*cos(th);w;al;ar];  
 
% algebraic function
z_fun = [];                   

% implicit ODE: impl_f = 0
xdot = SX.sym('xdot',nx,1);
impl_f = xdot - x_dot;        
     
%% Objectives and constraints

% inner objectives
h = [z;y;th;vl;vr;al;ar];
hN = h(1:nyN);

obst_c = [0.6;1.5]; % DAVIDE: change it also in InitData > reference generation
obst_r = 0.1;

% outer objectives
obji = 0.5*(h-refs)'*diag(Q)*(h-refs) + APF.circle(h(1:2),obst_c,obst_r);
objN = 0.5*(hN-refN)'*diag(QN)*(hN-refN) + APF.circle(h(1:2),obst_c,obst_r);

obji_GGN = 0.5*(aux-refs)'*(aux-refs);
objN_GGN = 0.5*(auxN-refN)'*(auxN-refN);

% general inequality constraints
general_con = [];
general_con_N = [];

%% NMPC discretizing time length [s]

Ts_st = 1/50; % shooting interval time