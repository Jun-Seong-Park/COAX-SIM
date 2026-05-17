function CoaX_Ani_Final_Corrected(Position, Angle, TiltAngle, i_f, dt, name)
    %% 1. Scene & Environment Setup
    screenSize = get(0, 'ScreenSize');
    fig1 = figure('Position', [100, 100, 1080, 720], 'Color', 'w'); 
    ax = gca; hold on; grid on; axis equal;
    
    xlabel('X [m]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontSize', 12, 'FontWeight', 'bold');
    set(gca, 'FontSize', 10, 'LineWidth', 1.2);
    view(3);

    %% 2. Lighting & Material
    light('Position', [-50 -50 50], 'Style', 'local');
    light('Position', [50 50 100], 'Style', 'infinite');
    lighting gouraud; material dull;

    %% 3. Drone Parts Modeling
    
    % --- [A] Main Body (White Fuselage) ---
    [vb, fb] = GetFuselageMesh(0.5, 0.35, 0.25); 
    h_body = patch('Vertices', vb, 'Faces', fb, ...
        'FaceColor', 'w', 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.6);
    material(h_body, 'shiny');

    % --- [B] Rotor Mast (Vertical Cylinder) - FIXED ---
    % Error fixed: Ensure the cylinder is strictly vertical along Z-axis
    r_mast = 0.03;   % Radius
    z_start = -0.15; % Start inside body
    z_end = 0.65;    % End above upper rotor
    
    % Get corrected mesh coordinates (X, Y, Z)
    [mx, my, mz] = GetMastMesh(r_mast, z_start, z_end);
    
    % Plot surface using correct X, Y, Z data
    h_mast = surface(mx, my, mz, 'FaceColor', 'k', 'EdgeColor', 'none');

    % --- [C] Skid Landing Gear ---
    h_skids = GetSkidGear(0.35, 0.6, -0.4);

    % --- [D] Rotor Blades ---
    [vp, fp] = GetBladeMesh(1.3);
    h_prop_up = patch('Vertices', vp, 'Faces', fp, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    h_prop_dw = patch('Vertices', vp, 'Faces', fp, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.95);

    %% 4. Assembly (Parent-Child Hierarchy)
    t_drone = hgtransform('Parent', ax);
    
    % Attach static parts to drone body group
    set(h_body, 'Parent', t_drone);
    set(h_mast, 'Parent', t_drone); % Attached correctly now
    for k = 1:length(h_skids), set(h_skids(k), 'Parent', t_drone); end
    
    % Rotor transforms
    t_prop_up = hgtransform('Parent', t_drone);
    t_prop_dw = hgtransform('Parent', t_drone);
    set(h_prop_up, 'Parent', t_prop_up);
    set(h_prop_dw, 'Parent', t_prop_dw);
    
    prop_up_z = 0.55; prop_dw_z = 0.30;
    set(t_prop_up, 'Matrix', makehgtform('translate', [0, 0, prop_up_z]));
    
    t_prop_dw_tilt = hgtransform('Parent', t_drone);
    set(t_prop_dw_tilt, 'Matrix', makehgtform('translate', [0, 0, prop_dw_z]));
    set(h_prop_dw, 'Parent', t_prop_dw_tilt);

    %% 5. Shadow
    h_shadow = patch('XData', [], 'YData', [], 'ZData', [], ...
        'FaceColor', 'k', 'FaceAlpha', 0.25, 'EdgeColor', 'none');

    %% 6. Animation Loop
    writerObj = VideoWriter(name, 'MPEG-4'); writerObj.FrameRate = 30; open(writerObj);
    prop_angle = 0; camup([0 0 1]); 
    
    for i = 1:15:i_f 
        p = Position(:, i); ang = Angle(:, i); tilt = TiltAngle(:, i);
        
        % 1. Move Drone
        M_pos = makehgtform('translate', p);
        M_rot = makehgtform('zrotate', ang(3)) * makehgtform('yrotate', ang(2)) * makehgtform('xrotate', ang(1));
        set(t_drone, 'Matrix', M_pos * M_rot);
        
        % 2. Animate Rotors
        prop_angle = prop_angle + 0.5; 
        set(t_prop_up, 'Matrix', makehgtform('translate', [0,0,prop_up_z]) * makehgtform('zrotate', prop_angle));
        M_tilt = makehgtform('yrotate', -tilt(2)) * makehgtform('xrotate', -tilt(1));
        M_spin = makehgtform('zrotate', -prop_angle);
        set(t_prop_dw_tilt, 'Matrix', makehgtform('translate', [0,0,prop_dw_z]) * M_tilt * M_spin);
        
        % 3. Update Shadow & Camera
        theta_s = linspace(0, 2*pi, 30); r_s = 1.0;
        set(h_shadow, 'XData', p(1) + r_s*cos(theta_s), 'YData', p(2) + r_s*sin(theta_s), 'ZData', zeros(size(theta_s)) + 0.01);
        camtarget(p'); 
        xlim([p(1)-3, p(1)+3]); 
        ylim([p(2)-3, p(2)+3]); 
        zlim([p(3)-3, p(3)+3]);
        
        drawnow; frame = getframe(fig1); writeVideo(writerObj, frame);
    end
    close(writerObj); close(fig1);
end

%% === Helper Functions ===

function [v, f] = GetFuselageMesh(lx, ly, lz)
    [x, y, z] = ellipsoid(0,0,0, lx, ly, lz, 20);
    z(z > 0) = z(z > 0) * 0.8; v = [x(:) y(:) z(:)]; f = convhull(v);
end

% === [Fixed Function] Correct Vertical Cylinder ===
function [x, y, z] = GetMastMesh(r, z_start, z_end)
    [x, y, z_cyl] = cylinder(r, 16); % Generate unit cylinder
    z = z_cyl * (z_end - z_start) + z_start; % Scale only Z-axis
    % Previous error: I was returning Y as Z, causing the tilt.
    % Now returning [x, y, z] correctly.
end

function h_group = GetSkidGear(width_half, length_half, z_floor)
    h_group = []; r_tube = 0.025;
    [xc, yc, zc] = cylinder(r_tube, 10); zc = (zc - 0.5) * (length_half * 2.5);
    h1 = surface(zc, xc - width_half, yc + z_floor, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none');
    h2 = surface(zc, xc + width_half, yc + z_floor, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none');
    h_struts = []; strut_x_pos = [-length_half/2, length_half/2];
    for x_p = strut_x_pos, for y_dir = [-1, 1]
            [xs, ys, zs] = cylinder(r_tube, 8); zs = zs * abs(z_floor) - 0.1; 
            h_s = surface(xs + x_p, ys + (y_dir * width_half), zs + z_floor, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none');
            h_struts = [h_struts, h_s];
    end, end
    h_group = [h1, h2, h_struts];
end

function [v, f] = GetBladeMesh(r)
    w_tip = 0.06 * r; w_hub = 0.12 * r; t = 0.01;
    v1 = [r, w_tip/2, 0; r, -w_tip/2, 0; 0, -w_hub/2, 0; 0, w_hub/2, 0; r, w_tip/2, t; r, -w_tip/2, t; 0, -w_hub/2, t; 0, w_hub/2, t];
    f1 = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    v2 = [-v1(:,1), v1(:,2), v1(:,3)]; f2 = f1 + 8; v = [v1; v2]; f = [f1; f2];
end