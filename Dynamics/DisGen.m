function Disturbance = DisGen(t, p)
% DisGen  Generate external disturbance force/moment at time t.
%
%   Disturbance = DisGen(t, p)
%
%   Model (per group g = {force, moment}):
%       d_g(t) = dis_bias(g) + dis_max(g) * sin(dis_freq(g) * t)
%   applied identically to the 3 axes of that group, but only while t
%   falls inside one of the active windows in p.dis_time.
%
%   Inputs:
%     t   scalar : current simulation time [s]
%     p   struct : uses the fields below
%         .dis_mode  scalar : 0 = no disturbance (master off), 1 = on
%         .dis_bias  [2x1]  : DC bias        [force_bias; moment_bias]
%         .dis_max   [2x1]  : sine amplitude  [force_amp;  moment_amp]
%         .dis_freq  [2x1]  : sine frequency  [force_freq; moment_freq] [rad/s]
%         .dis_time  [Nx2]  : active windows, each row [t_start, t_end].
%                             [0 Inf] = full duration. [] = full (fallback).
%
%   Output:
%     Disturbance [6x1] : [Fx; Fy; Fz; Mx; My; Mz]

    % --- Master switch: disturbance disabled ---
    if p.dis_mode == 0
        Disturbance = zeros(6,1);
        return
    end

    % --- Time gating: is t inside any active window? ---
    windows = p.dis_time;
    if isempty(windows)
        active = true;                              % empty -> full
    else
        in_window = (t >= windows(:,1)) & (t <= windows(:,2));
        active    = any(in_window);                 % [] -> any([]) = false
    end

    if ~active
        Disturbance = zeros(6,1);
        return
    end

    % --- Disturbance model: DC bias + sinusoid (freq in rad/s) ---
    bias = p.dis_bias;
    amp  = p.dis_max;
    freq = p.dis_freq;

    force_val  = bias(1) + amp(1) * sin(freq(1) * t);   % force,  all 3 axes
    moment_val = bias(2) + amp(2) * sin(freq(2) * t);   % moment, all 3 axes

    Force_Disturbance  = force_val  * ones(3,1);
    Moment_Disturbance = moment_val * ones(3,1);

    Disturbance = [Force_Disturbance; Moment_Disturbance];
end
