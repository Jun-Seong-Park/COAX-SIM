%% Coaxial Rotor Drone Simulator
% Jun Seong Park | jsp991204@gmail.com
clc; clear all; close all

%% Add all subdirectories
sim_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(sim_dir,'Controller'), ...
        fullfile(sim_dir,'Dynamics'), ...
        fullfile(sim_dir,'Trajectory'), ...
        fullfile(sim_dir,'Scenario'), ...
        fullfile(sim_dir,'Plot'), ...
        fullfile(sim_dir,'Animation'))

%% Mode Selection Scenario & Control
%   scenario |  description       | recommended tf
%   ---------|--------------------|--------------
%       0    |  hover             |  20s
%       1    |  box + yaw         |  70s
%       2    |  circle R=5m       |  80s
%       3    |  circle + heading  |  80s
%       4    |  RRT CSV           |  140s
p.scenario_id    = 3;
p.ctrl_mode      = 3;   % 1: BSC+DOB  2: BSC+CBF  3: BSC+CBF+DOB  4: MPC
p.rotor_dyn_mode = 3;   % 1: instant  2: first-order lag  3: Schafroth rotor + servo lag
p.dis_mode       = 3;   % 0: off, 1: const, 2: sin(t), 3: sin(0.1t), 4: sin(10t)
p.obs_mode       = 1;   % 1: true state + true disturbance  2: noisy + LDOB



%% Time
p.t0      = 0;
p.tf      = 80;
p.dt_sim   = 0.001;
p.dt_ctrl  = 0.002;          % control loop period [s] (500Hz)
p.dt_rot   = 0.005;          % rotor ESC update period [s] (200Hz)
p.dt_servo = 0.02;           % servo update period [s] (50Hz)
p.dt_save  = 0.01;

%% Coaxial Rotor Drone settings
p.g    = 9.81;
p.Mass = 11.5;
p.Ixx  = 0.2203;  p.Iyy = 0.2567;  p.Izz = 0.1056;
p.Inertia = diag([p.Ixx, p.Iyy, p.Izz]);
p.r_cd = 0.2;     % CoG-to-lower-rotor distance [m]
p.D    = 0.88;    % rotor diameter D [m] (radius = 0.44m)
p.rho  = 1.225;   % air density [kg/m^3]
% Hover aero coefficients (standard convention: T = cT * rho * n^2 * D^4, n=rev/s)
p.cT_hover = 0.091659;
p.cQ_hover = 0.003523;

% Coaxial interference ratio (lower/upper, <1 = lower rotor loss)
p.kT_interf = 0.85;        % thrust: lower rotor 15% loss
p.kQ_interf = 0.90;        % torque: lower rotor 10% loss

% Convert to k,gamma: T = k*Omega^2, Q = gamma*Omega^2  (Omega = rad/s)
%   k = cT_hover * rho * D^4 / (2*pi)^2
k_total     = p.cT_hover * p.rho * p.D^4 / (2*pi)^2;
gamma_total = p.cQ_hover * p.rho * p.D^5 / (2*pi)^2;

% Split into upper/lower by interference ratio
k_up = k_total / (1 + p.kT_interf);
k_dw = p.kT_interf * k_up;
g_up = gamma_total / (1 + p.kQ_interf);
g_dw = p.kQ_interf * g_up;

p.k     = [k_up; k_dw];
p.gamma = [g_up; g_dw];

% Derive c_T, c_Q in code convention: T = c_T * pi * rho * D^4 * Omega^2
p.c_T   = p.k     / (pi * p.rho * p.D^4);
p.c_Q   = p.gamma / (pi * p.rho * p.D^5);

p.Omega_hover = sqrt(p.Mass * p.g / (p.k(1) + p.k(2)));

% Servo time constant
p.tau_sw         = 0.05;  % swashplate servo time constant [s] (mode 2,3)
p.tau_rot        = 0.1;   % rotor motor time constant [s] (mode 2)

% Schafroth rotor dynamics (mode 3)
p.J_rot          = 0.01;  % rotor moment of inertia [kg·m^2]
p.K_motor        = p.J_rot / p.tau_rot;  % motor gain [Nm/(rad/s)], consistent with tau_rot


%% Constraints
p.angle_max     = [deg2rad(5); deg2rad(5); deg2rad(360)]; %% caution servo dead zone
p.angle_dot_max = [deg2rad(10); deg2rad(10); deg2rad(30)];
p.Omega_max     = 3000 * 2*pi/60;   % [rpm -> rad/s]
p.tilt_max      = [deg2rad(15); deg2rad(15)];
p.rot_max       = [p.tilt_max; p.Omega_max];

%% Controller gains — Position BSC
p.K1_pos = [1; 1; 1];
p.K2_pos = [3; 3; 3];

%% Controller gains — Attitude BSC (shared by mode 1,2,3)
p.K1_bsc = [15; 15; 15];
p.K2_bsc = [5; 5; 5];

%% Controller gains — CBF (mode 2,3 only)
p.P1_cbf   = 10;                % HOCBF class-K gain (B1, 1st layer)
p.P2_cbf   = 10;                % HOCBF class-K gain (B1, 2nd layer)
p.P3_cbf   = 10;                % HOCBF class-K gain (B3, 3rd layer)
p.tau_filt = 0.02;              % 1st-order filter time constant [s]

%% Observer
p.L_dob     = 0.9;             % LDOB attitude gain (obs_mode 2,3)
p.L_dob_pos = 0.9;             % LDOB position gain (obs_mode 2,3)

%% Sensor noise (obs_mode 3 only, 3-sigma = max)
p.sigma_pos  = [0.003; 0.003; 0.003];          % 3-sigma ≈ 1cm
p.sigma_vel  = [0.003; 0.003; 0.003];          % 3-sigma ≈ 1cm/s
p.sigma_acc  = [0.003; 0.003; 0.003];          % 3-sigma ≈ 0.03 m/s^2
p.sigma_att  = deg2rad([0.03; 0.03; 0.03]);    % 3-sigma ≈ 0.1 deg
p.sigma_gyro = deg2rad(0.03);                  % 3-sigma ≈ 0.1 deg/s

%% Disturbance
p.Dis_max  = [0.1; 0.02];

%% Run simulation
results = run_sim(p);
metrics = make_metrics(results, p);

%% Save
writematrix([results.t', results.X_s'], 'results.csv');

%% Plot
plot_results(results, p);

%% Animation
% animate_coax(results, p, 'wide', 6, 'sim_result');  % input: results, params, view(track, top, side, wide), speed, save name

