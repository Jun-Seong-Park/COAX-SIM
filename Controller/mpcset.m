function [x_mpc, mpcobj] = mpcset(state_max, p)
% mpcset  Create MPC controller object for attitude control
%   [x_mpc, mpcobj] = mpcset(state_max, p)
%
%   Inputs:
%     state_max [3x3] : [angle_max, angle_dot_max, Torque_max]
%     p         struct: params (uses Ixx, Iyy, Izz, dt_ctrl)

A = [0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0];

B = [0 0 0;
     0 0 0;
     0 0 0;
     1/p.Ixx 0 0;
     0 1/p.Iyy 0;
     0 0 1/p.Izz];

C = eye(6);
D = zeros(6,3);

sys = ss(A, B, C, D);

max_phi   = state_max(1,1);
max_theta = state_max(2,1);
max_psi   = state_max(3,1);
max_p = state_max(1,2);
max_q = state_max(2,2);
max_r = state_max(3,2);
Torque_max = state_max(:,3);

mpcobj = mpc(sys, p.dt_ctrl);
mpcobj.PredictionHorizon = 40;
mpcobj.ControlHorizon = 8;
mpcobj.Weights.OutputVariables = [1 1 1 0 0 0];
mpcobj.Weights.ManipulatedVariables = [0.1 0.1 0.1];
mpcobj.Weights.ManipulatedVariablesRate = [1 1 1];

% Output variable constraints
mpcobj.OV(1).Min = -max_phi; mpcobj.OV(1).Max = max_phi;
mpcobj.OV(2).Min = -max_theta; mpcobj.OV(2).Max = max_theta;
mpcobj.OV(3).Min = -max_psi; mpcobj.OV(3).Max = max_psi;
mpcobj.OV(4).Min = -max_p; mpcobj.OV(4).Max = max_p;
mpcobj.OV(5).Min = -max_q; mpcobj.OV(5).Max = max_q;
mpcobj.OV(6).Min = -max_r; mpcobj.OV(6).Max = max_r;
for k = 1:6
    mpcobj.OV(k).MinECR = 0;
    mpcobj.OV(k).MaxECR = 0;
end

% Manipulated variable (torque) constraints
mpcobj.MV(1).Min = -Torque_max(1); mpcobj.MV(1).Max = Torque_max(1);
mpcobj.MV(2).Min = -Torque_max(2); mpcobj.MV(2).Max = Torque_max(2);
mpcobj.MV(3).Min = -Torque_max(3); mpcobj.MV(3).Max = Torque_max(3);

x_mpc = mpcstate(mpcobj);
end
