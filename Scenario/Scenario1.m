function X_d = Scenario1(t)
%% Scenario 1 — Box + Stop-Rotate-Go
% 10x10m box at 5m altitude
% Pattern per edge: stop at corner → yaw rotate 5s → straight move
% Timeline:
%   0~5s     hover at origin
%   5~15s    move (0,0)→(10,0), yaw=0
%   15~20s   stop at (10,0), yaw 0→90
%   20~30s   move (10,0)→(10,10), yaw=90
%   30~35s   stop at (10,10), yaw 90→180
%   35~45s   move (10,10)→(0,10), yaw=180
%   45~50s   stop at (0,10), yaw 180→270
%   50~60s   move (0,10)→(0,0), yaw=270
%   60s~     hover at origin, yaw=270

    z   = -5;
    L   = 10;

    if t < 5                                        % hover
        X_d = [0; 0; z; 0];

    elseif t < 15                                   % edge 1: +x
        r = (t - 5) / 10;
        X_d = [L*r; 0; z; 0];

    elseif t < 20                                   % corner 1: yaw 0→90
        r = (t - 15) / 5;
        X_d = [L; 0; z; r*pi/2];

    elseif t < 30                                   % edge 2: +y
        r = (t - 20) / 10;
        X_d = [L; L*r; z; pi/2];

    elseif t < 35                                   % corner 2: yaw 90→180
        r = (t - 30) / 5;
        X_d = [L; L; z; pi/2 + r*pi/2];

    elseif t < 45                                   % edge 3: -x
        r = (t - 35) / 10;
        X_d = [L*(1-r); L; z; pi];

    elseif t < 50                                   % corner 3: yaw 180→270
        r = (t - 45) / 5;
        X_d = [0; L; z; pi + r*pi/2];

    elseif t < 60                                   % edge 4: -y
        r = (t - 50) / 10;
        X_d = [0; L*(1-r); z; 3*pi/2];

    else                                            % done
        X_d = [0; 0; z; 3*pi/2];
    end
end
