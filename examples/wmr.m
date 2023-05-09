%------------------------------------------%

% Wheeled Mobile Robot (WMR) - Davide De Lazzari

%------------------------------------------%


%% Dimensions

nx=5;  % No. of differential states
nu=2;  % No. of controls
nz=0;  % No. of algebraic states
ny=7; % No. of outputs
nyN=5; % No. of outputs at the terminal point

% laser_samples = 360; % must be changed also in initialization_Simulink

% np=laser_samples*4; % No. of model parameters
np=10;

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
auxN     = SX.sym('auxN',nyN,1);    % auxilary variablez<

%% Dynamics

z = states(1); % z axis = horizontal
y = states(2); % y axis = vertical
th = states(3); % anticlockwise angle wrt z axis
v = states(4); % left wheel vel
w = states(5); % right wheel vel

vdot = controls(1); % left wheel acc
wdot = controls(2); % right wheel acc

% explicit ODE RHS
x_dot=[v*cos(th);v*sin(th);w;vdot;wdot];  
 
% algebraic function
z_fun = [];                   

% implicit ODE: impl_f = 0
xdot = SX.sym('xdot',nx,1);
impl_f = xdot - x_dot;

obstacles = params;
% obstacles(1) = 3;
% obstacles(2) = 0;
% obstacles(3) = 2.95;
% obstacles(4) = 0.05;
% obstacles(5) = 3.05;
% obstacles(6) = 0.05;

     
%% Objectives and constraints

% inner objectives
h = [z;y;th;v;w;vdot;wdot];
hN = h(1:nyN);

% outer objectives
obji = 0.5*(h-refs)'*diag(Q)*(h-refs) + obstacles_pen(h(1:2),obstacles,np);
objN = 0.5*(hN-refN)'*diag(QN)*(hN-refN) + obstacles_pen(h(1:2),obstacles,np);

obji_GGN = 0.5*(aux-refs)'*(aux-refs);
objN_GGN = 0.5*(auxN-refN)'*(auxN-refN);


general_con = [];
general_con_N = [];

%% NMPC discretizing time length [s]

Ts_st = 0.07 ;%1/15; % shooting interval time

%%

% PIERO:
% 1/28 * gaussian(model._x['v'],0.0002, 0.05)

% out_test = obstacles_pen([2.15,0],[3,0,1,0])

% function y = cost_test(pos2d,obstacles)
%     x_wc = pos2d(1); 
%     y_wc = pos2d(2);
%     xo = obstacles(1);
%     yo = 2;
%     y = -((x_wc - xo)^2 + (y_wc - yo)^2);
% end

function y = obstacles_pen(pos2d,obstacles,np)
    x_wc = pos2d(1);
    y_wc = pos2d(2);
    y = 0;
    for i = 1:2:np
        xo = obstacles(i);
%         xo = 3;
        yo = obstacles(i+1);
%         yo = 0;
        y = max(y,obst_pen(x_wc,y_wc,xo,yo));
%         y = y + obst_pen(x_wc,y_wc,xo,yo);
    end
end

function y = obst_pen(x_wc,y_wc,x_obst,y_obst)
    d = sqrt((x_wc - x_obst)^2 + (y_wc - y_obst)^2);
    y = pen(d);
end

function y = pen(d)
    costmap_resolution = 0.1;
    wc_radius = 0.8;
    d = d - wc_radius - costmap_resolution/2;
    c = 100; % steepness
    k = 100; % max penalty
    y = k*sigmoid(-d,c);
end

function y = sigmoid(x,c)
    y = 1/(1+exp(-x*c));
%     y = 0.1;
end