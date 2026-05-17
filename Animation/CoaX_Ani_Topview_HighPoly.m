function CoaX_Ani_Topview_HighPoly(POS_d_vec, Position, Angle, TiltAngle, i_f, dt, xlimit, ylimit, zlimit, name)
    % Coaxial Helicopter Animation (High-Fidelity Model, Fixed View)
    
    %% 1. 화면 및 환경 설정
    screenSize = get(0, 'ScreenSize');
    fig1 = figure('Position', [screenSize(3)/2-480, screenSize(4)/2-360, 960, 720], 'Color', 'w'); 
    
    ax = gca;
    grid on; axis equal;
    
    % 고정 프레임 설정
    xlim(xlimit);       
    ylim(ylimit);       
    zlim(zlimit);
    
    xlabel('X [m]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontSize', 12, 'FontWeight', 'bold');
    title('Coaxial eVTOL Trajectory Tracking', 'FontSize', 14);
    
    view(43, 24); 
    hold on;

    %% 2. 조명 설정
    light('Position', [xlimit(2) ylimit(1) zlimit(2)], 'Style', 'local');
    light('Position', [xlimit(1) ylimit(2) zlimit(2)], 'Style', 'local');
    lighting gouraud; 
    material dull;

    %% 3. [핵심 수정] Reference Path 미리 그리기 (Pre-draw)
    % 루프 밖에서 전체 경로를 한 번에 그립니다.
    % 이미 전처리를 하셨다니 POS_d_vec 그대로 사용합니다.
    plot3(POS_d_vec(1,:), POS_d_vec(2,:), POS_d_vec(3,:), ...
        'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference Path');

    %% 4. 드론 부품 모델링
    % [A] Main Body
    [vb, fb] = GetFuselageMesh(0.5, 0.35, 0.25); 
    h_body = patch('Vertices', vb, 'Faces', fb, ...
        'FaceColor', 'w', 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.6);
    material(h_body, 'shiny');

    % [B] Rotor Mast
    r_mast = 0.03; z_start = -0.15; z_end = 0.65;
    [vm, fm] = GetMastMesh(r_mast, z_start, z_end);
    h_mast = surface(vm, fm, vm*0, 'FaceColor', 'k', 'EdgeColor', 'none'); 
    h_mast.ZData = fm * (z_end - z_start) + z_start; 

    % [C] Skid Gear
    h_skids = GetSkidGear(0.35, 0.6, -0.4);

    % [D] Rotor Blades
    [vp, fp] = GetBladeMesh(1.3);
    h_prop_up = patch('Vertices', vp, 'Faces', fp, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    h_prop_dw = patch('Vertices', vp, 'Faces', fp, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.95);

    % [E] Shadow
    h_shadow = patch('XData', [], 'YData', [], 'ZData', [], ...
        'FaceColor', 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');

    %% 5. 조립 (Hierarchy)
    t_drone = hgtransform('Parent', ax);
    
    set(h_body, 'Parent', t_drone);
    set(h_mast, 'Parent', t_drone);
    for k = 1:length(h_skids), set(h_skids(k), 'Parent', t_drone); end
    
    t_prop_up = hgtransform('Parent', t_drone);
    t_prop_dw = hgtransform('Parent', t_drone);
    set(h_prop_up, 'Parent', t_prop_up);
    set(h_prop_dw, 'Parent', t_prop_dw);
    
    prop_up_z = 0.55; 
    prop_dw_z = 0.30;
    set(t_prop_up, 'Matrix', makehgtform('translate', [0, 0, prop_up_z]));
    
    t_prop_dw_tilt = hgtransform('Parent', t_drone);
    set(t_prop_dw_tilt, 'Matrix', makehgtform('translate', [0, 0, prop_dw_z]));
    set(h_prop_dw, 'Parent', t_prop_dw_tilt);

    %% 6. Video Writer & Loop
    writerObj = VideoWriter(name, 'MPEG-4');
    writerObj.FrameRate = 30; 
    open(writerObj);
    
    timeText = text(0.95, 0.05, '', 'FontSize', 12, 'Color', 'k', ...
        'FontWeight', 'bold', 'Units', 'normalized', 'HorizontalAlignment', 'right');

    prop_angle = 0;
    
    % 실제 궤적(Actual Path)용 라인 (이건 계속 그려져야 하므로 초기화만)
    h_act_path = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Drone Path');
    
    % 범례 표시 (미리 그려둔 Reference Path 포함됨)
    legend('show', 'Location', 'northeast');

    for i = 1:10:i_f 
        
        p = Position(:, i);
        ang = Angle(:, i);
        tilt = TiltAngle(:, i);
        
        % 1. 드론 이동
        M_pos = makehgtform('translate', p);
        M_rot = makehgtform('zrotate', ang(3)) * ...
                makehgtform('yrotate', ang(2)) * ...
                makehgtform('xrotate', ang(1));
        set(t_drone, 'Matrix', M_pos * M_rot);
        
        % 2. 로터 회전
        prop_angle = prop_angle + 0.6;
        set(t_prop_up, 'Matrix', makehgtform('translate', [0,0,prop_up_z]) * makehgtform('zrotate', prop_angle));
        
        M_tilt = makehgtform('yrotate', -tilt(2)) * makehgtform('xrotate', -tilt(1));
        M_spin = makehgtform('zrotate', -prop_angle);
        set(t_prop_dw_tilt, 'Matrix', makehgtform('translate', [0,0,prop_dw_z]) * M_tilt * M_spin);
        
        % 3. 그림자
        theta_s = linspace(0, 2*pi, 20); r_s = 0.8;
        set(h_shadow, 'XData', p(1) + r_s*cos(theta_s), ...
                      'YData', p(2) + r_s*sin(theta_s), ...
                      'ZData', zeros(size(theta_s)) + 0.01);
                  
        % 4. 실제 궤적 업데이트 (Actual Path)
        set(h_act_path, 'XData', Position(1, 1:i), ...
                        'YData', Position(2, 1:i), ...
                        'ZData', Position(3, 1:i));
        
        % *주의* Reference Path 업데이트 코드는 삭제했습니다 (이미 위에서 다 그렸음)

        % 5. 시간 텍스트
        set(timeText, 'String', sprintf('Time: %.2f s', i * dt));
        
        drawnow;
        frame = getframe(fig1);
        writeVideo(writerObj, frame);
    end
    
    close(writerObj);
    close(fig1);
end

%% === Helper Functions ===
function [v, f] = GetFuselageMesh(lx, ly, lz)
    [x, y, z] = ellipsoid(0,0,0, lx, ly, lz, 20);
    z(z > 0) = z(z > 0) * 0.8; v = [x(:) y(:) z(:)]; f = convhull(v);
end
function [x, y] = GetMastMesh(r, z_start, z_end)
    [x, y, ~] = cylinder(r, 16); 
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