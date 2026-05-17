function draw_obstacles(ax)
% draw_obstacles  Load obstacles.csv and render as 3D boxes
%   draw_obstacles(ax)
%
%   Searches for obstacles.csv in current folder, then Trajectory/.
%   Draws translucent AABB boxes into the given axes.

    csv_file = 'obstacles.csv';
    if ~isfile(csv_file)
        csv_file = fullfile('Trajectory', 'obstacles.csv');
    end
    if ~isfile(csv_file)
        warning('draw_obstacles: obstacles.csv not found');
        return;
    end

    obs = readmatrix(csv_file);     % [xmin ymin zmin xmax ymax zmax]

    for k = 1:size(obs, 1)
        b = obs(k,:);
        v = [b(1) b(2) b(3); b(4) b(2) b(3); b(4) b(5) b(3); b(1) b(5) b(3);
             b(1) b(2) b(6); b(4) b(2) b(6); b(4) b(5) b(6); b(1) b(5) b(6)];
        f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

        patch(ax, 'Vertices', v, 'Faces', f, ...
            'FaceColor', [0.75 0.75 0.85], 'FaceAlpha', 0.5, ...
            'EdgeColor', [0.4 0.4 0.5], 'LineWidth', 0.5, ...
            'HandleVisibility', 'off');
    end
end
