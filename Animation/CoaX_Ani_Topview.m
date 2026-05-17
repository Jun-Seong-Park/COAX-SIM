function CoaX_Ani_Topview(POS_d_vec, Position, Angle, TiltAngle, i_f, dt, xlimit, ylimit, zlimit, name)

    % Coaxial Helicopter Animation
    % Position: [x, y, z] positions over time
    % Angle: [roll, pitch, yaw] angles over time
    % TiltAngle: [alpha, beta] angles for lower rotor over time
    % i_f: final iteration
    % dt: time step for iteration
    % name: name of the output video file

    %% Define design parameters
    b = 1;  % Body length (cube side length)
    H = 1;  % Body height
    r_p = 1.5;  % Propeller radius
    prop_dw_offset = 0.75;  % Offset between body and lower propeller
    prop_up_offset = 1;  % Offset between body and upper propeller (center to top)

    %% Generate propeller coordinates
    theta_o = linspace(0, 2*pi);
    xp = r_p * cos(theta_o);
    yp = r_p * sin(theta_o);
    zp = zeros(1, length(theta_o));

    %% Define figure plot
    screenSize = get(0, 'ScreenSize');
    fig1 = figure('Position', [screenSize(3)/2-400, screenSize(4)/2-300, 800, 600]);  % Centered figure
    hg = gca;
    
    view(43,24);
    grid on;
    axis equal;
    title('Coaxial Helicopter Animation');
    xlim(xlimit);       
    ylim(ylimit);       
    zlim(zlimit);
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    
    hold on;

    %% Design helicopter body
    vertices = [-b/2, -b/2, -H/2;
                 b/2, -b/2, -H/2;
                 b/2,  b/2, -H/2;
                -b/2,  b/2, -H/2;
                -b/2, -b/2,  H/2;
                 b/2, -b/2,  H/2;
                 b/2,  b/2,  H/2;
                -b/2,  b/2,  H/2];
    faces = [1 2 3 4;
             5 6 7 8;
             1 2 6 5;
             2 3 7 6;
             3 4 8 7;
             4 1 5 8];
    helicopter(1) = patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'r');
    alpha(helicopter(1), 0.7);

    %% Design the shaft (middle rod)
    [xs, ys, zs] = cylinder(0.02);
    zs = zs * (prop_up_offset + H/2);
    helicopter(2) = surface(xs, ys, zs, 'FaceColor', 'k', 'EdgeColor', 'none');
    alpha(helicopter(2), 0.7);

    %% Design propellers
    lower_prop_z = prop_dw_offset;
    upper_prop_z = H/2 + prop_up_offset;
    helicopter(3) = patch(xp, yp, zp + lower_prop_z, 'c', 'LineWidth', 0.5);
    alpha(helicopter(3), 0.3);
    helicopter(4) = patch(xp, yp, zp + upper_prop_z, 'b', 'LineWidth', 0.5);
    alpha(helicopter(4), 0.3);

    %% Add points at the center of propellers
    lower_prop_center = plot3(0, 0, lower_prop_z, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
    upper_prop_center = plot3(0, 0, upper_prop_z, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);

    %% Create a group object and parent surface
    combinedobject = hgtransform('Parent', hg);
    set(helicopter, 'Parent', combinedobject);
    set(upper_prop_center, 'Parent', combinedobject);  % Attach upper propeller center to combined object

    % Create a separate transform for the lower propeller's position and tilt
    lower_propeller_transform = hgtransform('Parent', combinedobject);
    set(lower_propeller_transform, 'Matrix', makehgtform('translate', [0, 0, lower_prop_z]));
    set([helicopter(3), lower_prop_center], 'Parent', lower_propeller_transform);  % Attach lower propeller and center point

    %% Set up the video writer
    writerObj = VideoWriter(name, 'MPEG-4');
    open(writerObj);

     %% Add time text
    timeText = text(0.95, 0.05, '', 'FontSize', 12, 'Color', 'k', 'FontWeight', 'bold', 'Units', 'normalized', 'HorizontalAlignment', 'right');

    %% Animation loop
    for i = 1:50:i_f
        % Translation matrix
        translation = makehgtform('translate', [Position(1,i), Position(2,i), Position(3,i)]);

        % Rotation matrices
        rotation1 = makehgtform('xrotate', Angle(1,i));
        rotation2 = makehgtform('yrotate', Angle(2,i));
        rotation3 = makehgtform('zrotate', Angle(3,i));

        % Tilt angle matrices for lower propeller
        tilt1 = makehgtform('xrotate', TiltAngle(1,i));
        tilt2 = makehgtform('yrotate', TiltAngle(2,i));

        % Update transformation matrix for the helicopter
        set(combinedobject, 'Matrix', translation * rotation3 * rotation2 * rotation1);

        % Update lower propeller tilt only
        set(lower_propeller_transform, 'Matrix', makehgtform('translate', [0, 0, lower_prop_z]) * tilt2 * tilt1 * makehgtform('translate', [0, 0, -lower_prop_z]));



        % Plot position trace
        plot3(Position(1,1:i), Position(2,1:i), Position(3,1:i), 'b', 'LineWidth', 1);
        hold on
        plot3(POS_d_vec(1,1:i), POS_d_vec(2,1:i), -POS_d_vec(3,1:i),'r','LineWidth', 1 );

       
        % Update time text
        currentTime = i * dt; % Current time in seconds
        set(timeText, 'String', sprintf('Time: %.2f s', currentTime));
        
        % Capture the frame for the video
        frame = getframe(fig1);
        writeVideo(writerObj, frame);

        drawnow;
        pause(dt);
    end

    % Close the video writer
    close(writerObj);
    hold on

    
end
