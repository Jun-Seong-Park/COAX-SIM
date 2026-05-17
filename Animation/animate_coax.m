function animate_coax(results, p, view_mode, speed, filename)
% animate_coax  3D trajectory tracking animation for coaxial drone
%   animate_coax(results, p)
%   animate_coax(results, p, view_mode, speed, filename)
%
%   Inputs:
%     results   struct from run_sim (X_s, X_d_s, t, etc.)
%     p         params struct
%     view_mode (optional) 'wide' | 'track' | 'top' | 'side', default: 'wide'
%     speed     (optional) playback speed multiplier, default: 1 (realtime)
%     filename  (optional) video filename, default: no video saved

    if nargin < 3 || isempty(view_mode), view_mode = 'wide'; end
    if nargin < 4 || isempty(speed), speed = 1; end
    if nargin < 5, filename = ''; end

    %% Extract & convert NED → ENU (flip z for display)
    X_s   = results.X_s;
    X_d_s = results.X_d_s;
    n     = size(X_s, 2);
    dt    = p.dt_save;

    % X_s(3,:) and X_d_s(3,:) already flipped in run_sim (stored as ENU)
    Position  = X_s(1:3,:);
    Angle     = X_s(4:6,:);
    TiltAngle = X_s(13:14,:);
    Ref       = X_d_s(1:3,:);

    %% Scene setup
    fig = figure('Position', [100 100 1280 720], 'Color', 'w');
    ax = gca; hold on; grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Coaxial Drone 3D Trajectory Tracking', 'FontSize', 14, 'FontWeight', 'bold');
    % Precompute full data range for wide/top/side views
    all_pts = [Position, Ref];
    margin = 2;
    x_range = [min(all_pts(1,:))-margin, max(all_pts(1,:))+margin];
    y_range = [min(all_pts(2,:))-margin, max(all_pts(2,:))+margin];
    z_range = [min(all_pts(3,:))-margin, max(all_pts(3,:))+margin];

    % Initial view angle
    switch view_mode
        case 'wide',  view(3);          % full overview, fixed range
        case 'track', view(3);          % camera follows drone
        case 'top',   view(0, 90);      % XY plane (top-down)
        case 'side',  view(0, 0);       % XZ plane (side)
        otherwise,    view(3);
    end

    %% Lighting
    light('Position', [-50 -50 50], 'Style', 'local');
    light('Position', [50 50 100], 'Style', 'infinite');
    lighting gouraud; material dull;

    %% Obstacles (scenario 4 only)
    if p.scenario_id == 4, draw_obstacles(ax); end

    %% Reference trajectory
    plot3(Ref(1,:), Ref(2,:), Ref(3,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'desired');
    h_trail = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', 'actual');
    legend('Location', 'best');

    %% Drone model
    [vb, fb] = GetFuselageMesh(0.5, 0.35, 0.25);
    h_body = patch('Vertices', vb, 'Faces', fb, ...
        'FaceColor', 'w', 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.6);
    material(h_body, 'shiny');

    % Canopy window (dark glass on front face)
    h_canopy = GetCanopy(0.5, 0.35, 0.25);

    r_mast = 0.03;
    [mx, my, mz] = GetMastMesh(r_mast, -0.15, 0.65);
    h_mast = surface(mx, my, mz, 'FaceColor', 'k', 'EdgeColor', 'none');

    h_skids = GetSkidGear(0.35, 0.6, -0.4);

    [vp, fp] = GetBladeMesh(1.3);
    h_prop_up = patch('Vertices', vp, 'Faces', fp, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    h_prop_dw = patch('Vertices', vp, 'Faces', fp, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.95);

    %% Assembly
    % Scale drone for large-scale scenarios (e.g. RRT)
    if p.scenario_id == 4, drone_scale = 10; else, drone_scale = 1; end

    t_drone = hgtransform('Parent', ax);
    t_scale = hgtransform('Parent', t_drone);
    set(t_scale, 'Matrix', makehgtform('scale', drone_scale));

    set(h_body, 'Parent', t_scale);
    set(h_canopy, 'Parent', t_scale);
    set(h_mast, 'Parent', t_scale);
    for k = 1:length(h_skids), set(h_skids(k), 'Parent', t_scale); end

    t_prop_up = hgtransform('Parent', t_scale);
    set(h_prop_up, 'Parent', t_prop_up);
    prop_up_z = 0.55; prop_dw_z = 0.30;
    set(t_prop_up, 'Matrix', makehgtform('translate', [0 0 prop_up_z]));

    t_prop_dw_tilt = hgtransform('Parent', t_scale);
    set(t_prop_dw_tilt, 'Matrix', makehgtform('translate', [0 0 prop_dw_z]));
    set(h_prop_dw, 'Parent', t_prop_dw_tilt);

    %% Shadow
    h_shadow = patch('XData', [], 'YData', [], 'ZData', [], ...
        'FaceColor', 'k', 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    %% Timer text
    timeText = text(0.85, 0.1, 'Time: 0.00 s', 'Units', 'normalized', ...
        'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k', ...
        'BackgroundColor', [1 1 1 0.7], 'EdgeColor', 'k');

    %% Video writer
    save_video = ~isempty(filename);
    if save_video
        if ~endsWith(filename, '.mp4', 'IgnoreCase', true)
            filename = [filename '.mp4'];
        end
        writerObj = VideoWriter(filename, 'MPEG-4');
        writerObj.FrameRate = 30;
        writerObj.Quality = 95;
        open(writerObj);
    end

    %% Fix viewport for non-track modes (set once, never touch in loop)
    if ~strcmp(view_mode, 'track')
        xlim(x_range); ylim(y_range); zlim(z_range);
    end

    %% Animation loop
    prop_angle = 0;
    camup([0 0 1]);
    fps = 30;
    % video_duration = sim_time / speed
    % total_frames = video_duration * fps
    % step = n / total_frames
    sim_time = (n-1) * dt;
    total_frames = max(1, round(sim_time / speed * fps));
    step = max(1, round(n / total_frames));
    fprintf('Animation: %.0fs sim, %dx speed, %d frames, step=%d\n', sim_time, speed, total_frames, step);

    for i = 1:step:n
        pos = Position(:,i);
        ang = Angle(:,i);
        tilt = TiltAngle(:,i);

        % Move drone
        M_pos = makehgtform('translate', pos);
        M_rot = makehgtform('zrotate', ang(3)) * makehgtform('yrotate', ang(2)) * makehgtform('xrotate', ang(1));
        set(t_drone, 'Matrix', M_pos * M_rot);

        % Spin rotors
        prop_angle = prop_angle + 0.5;
        set(t_prop_up, 'Matrix', makehgtform('translate', [0 0 prop_up_z]) * makehgtform('zrotate', prop_angle));
        M_tilt = makehgtform('yrotate', -tilt(2)) * makehgtform('xrotate', -tilt(1));
        set(t_prop_dw_tilt, 'Matrix', makehgtform('translate', [0 0 prop_dw_z]) * M_tilt * makehgtform('zrotate', -prop_angle));

        % Trail
        set(h_trail, 'XData', Position(1,1:i), 'YData', Position(2,1:i), 'ZData', Position(3,1:i));

        % Shadow
        th = linspace(0, 2*pi, 30);
        set(h_shadow, 'XData', pos(1)+cos(th), 'YData', pos(2)+sin(th), 'ZData', zeros(size(th))+0.01);

        % Camera (only track mode moves)
        if strcmp(view_mode, 'track')
            camtarget(pos');
            xlim([pos(1)-3 pos(1)+3]);
            ylim([pos(2)-3 pos(2)+3]);
            zlim([pos(3)-3 pos(3)+3]);
        end

        % Timer
        set(timeText, 'String', sprintf('Time: %.2f s', (i-1)*dt));

        drawnow;
        if save_video
            try writeVideo(writerObj, getframe(fig)); catch, end
        end
    end

    if save_video
        close(writerObj);
        fprintf('Video saved: %s\n', filename);
    end
end

%% === Helper Functions ===

function h = GetCanopy(lx, ly, lz)
% Among-us style visor: large, top-heavy, wraps around sides
%   Covers upper front + 20% of sides, bottom edge at body midline
    n = 25;

    % Visor spans wide (90% of side) and tall (top 70% of body)
    wy = 0.90 * ly;         % wide: almost full side width
    wz_top = 0.85 * lz;     % extends high above center
    wz_bot = 0.15 * lz;     % slight dip below center

    % Semi-ellipse grid: theta from -20deg(bottom) to 200deg(top+sides)
    r_grid  = linspace(0, 1, n);
    th_grid = linspace(-0.3, pi+0.3, n*3);  % wrap around top, not bottom
    [RR, TH] = meshgrid(r_grid, th_grid);

    Y = wy * RR .* cos(TH);
    Z_raw = RR .* sin(TH);
    % Scale asymmetrically: top stretches more than bottom
    Z = zeros(size(Z_raw));
    Z(Z_raw >= 0) = Z_raw(Z_raw >= 0) * wz_top;
    Z(Z_raw <  0) = Z_raw(Z_raw <  0) * wz_bot;
    Z = Z + 0.05 * lz;      % shift slightly up

    % Project onto ellipsoid surface
    r2 = (Y/ly).^2 + (Z/lz).^2;
    valid = r2 < 0.99;
    r2(~valid) = 0.99;
    X = lx * sqrt(1 - r2) + 0.006;

    % Mask out points that went behind the body midplane
    X(~valid) = NaN;

    h = surface(X, Y, Z, ...
        'FaceColor', [0.05 0.05 0.1], 'FaceAlpha', 0.9, ...
        'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.2, 'SpecularStrength', 1.0, ...
        'HandleVisibility', 'off');
    material(h, 'shiny');
end

function [v, f] = GetFuselageMesh(lx, ly, lz)
    [x, y, z] = ellipsoid(0,0,0, lx, ly, lz, 20);
    z(z > 0) = z(z > 0) * 0.8;
    v = [x(:) y(:) z(:)]; f = convhull(v);
end

function [x, y, z] = GetMastMesh(r, z_start, z_end)
    [x, y, z_cyl] = cylinder(r, 16);
    z = z_cyl * (z_end - z_start) + z_start;
end

function h_group = GetSkidGear(width_half, length_half, z_floor)
    r_tube = 0.025;
    [xc, yc, zc] = cylinder(r_tube, 10);
    zc = (zc - 0.5) * (length_half * 2.5);
    h1 = surface(zc, xc - width_half, yc + z_floor, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none');
    h2 = surface(zc, xc + width_half, yc + z_floor, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none');
    h_struts = [];
    for x_p = [-length_half/2, length_half/2]
        for y_dir = [-1, 1]
            [xs, ys, zs] = cylinder(r_tube, 8);
            zs = zs * abs(z_floor) - 0.1;
            h_s = surface(xs + x_p, ys + y_dir*width_half, zs + z_floor, ...
                'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none');
            h_struts = [h_struts, h_s];
        end
    end
    h_group = [h1, h2, h_struts];
end

function [v, f] = GetBladeMesh(r)
    w_tip = 0.06*r; w_hub = 0.12*r; t = 0.01;
    v1 = [r w_tip/2 0; r -w_tip/2 0; 0 -w_hub/2 0; 0 w_hub/2 0;
          r w_tip/2 t; r -w_tip/2 t; 0 -w_hub/2 t; 0 w_hub/2 t];
    f1 = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    v2 = [-v1(:,1), v1(:,2), v1(:,3)];
    f2 = f1 + 8;
    v = [v1; v2]; f = [f1; f2];
end
