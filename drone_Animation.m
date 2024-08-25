function drone_Animation(xe, ye, h, phi, theta, psi)
    % This function animates a QuadCopter's flight based on given trajectory and orientation.
    
   % Define design parameters
    D2R = pi/180;    % Conversion factor from degrees to radians
    b   = 0.6;       % Length of the total square cover by the whole body of quadcopter in meters
    a   = b/3;       % Length of small square base of quadcopter
    H   = 0.06;      % Height of drone in Z direction (6cm)
    H_m = H+H/2;     % Height of motor in Z direction (9cm)
    r_p = b/4;       % Radius of propeller
    
    % Define rotation matrix
    ro = 45 * D2R;   % Angle to rotate the base of quadcopter
    Ri = [cos(ro) -sin(ro) 0;  % Rotation matrix to rotate the coordinates of base 
          sin(ro)  cos(ro) 0;
          0        0       1];

    base_co = [-a/2  a/2  a/2 -a/2;  % Coordinates of Base 
               -a/2 -a/2  a/2  a/2;
                0    0    0    0];
    base = Ri * base_co;  % Rotate base coordinates by 45 degrees 

    to = linspace(0, 2 * pi);  % Create a vector for full circle
    xp = r_p * cos(to);  % X coordinates of propeller
    yp = r_p * sin(to);  % Y coordinates of propeller
    zp = zeros(1, length(to));  % Z coordinates of propeller (flat)
    
    % Define figure plot
    fig1 = figure('pos', [0 50 800 600]);  % Create figure window with specific size
    hg = gca;  % Get current axis handle
    view(68, 53);  % Set the view angle
    grid on;  % Enable grid
    axis equal;  % Set equal scaling for all axes
    xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([0 3.5]);  % Set axis limits
    title('Drone Animation');  % Set title of the plot
    xlabel('X[m]');  % Label for X axis
    ylabel('Y[m]');  % Label for Y axis
    zlabel('Z[m]');  % Label for Z axis
    hold(gca, 'on');  % Hold the current plot

    % Design different parts of the drone
    drone(1) = patch([base(1, :)], [base(2, :)], [base(3, :)], 'r');  % Base part 1
    drone(2) = patch([base(1, :)], [base(2, :)], [base(3, :) + H], 'r');  % Base part 2
    alpha(drone(1:2), 0.7);  % Set transparency

    [xcylinder, ycylinder, zcylinder] = cylinder([H/2 H/2]);  % Define a cylinder
    drone(3) = surface(b * zcylinder - b/2, ycylinder, xcylinder + H/2, 'facecolor', 'b');  % Vertical cylinder 1
    drone(4) = surface(ycylinder, b * zcylinder - b/2, xcylinder + H/2, 'facecolor', 'b');  % Vertical cylinder 2
    alpha(drone(3:4), 0.6);  % Set transparency

    drone(5) = surface(xcylinder + b/2, ycylinder, H_m * zcylinder + H/2, 'facecolor', 'r');  % Motor 1
    drone(6) = surface(xcylinder - b/2, ycylinder, H_m * zcylinder + H/2, 'facecolor', 'r');  % Motor 2
    drone(7) = surface(xcylinder, ycylinder + b/2, H_m * zcylinder + H/2, 'facecolor', 'r');  % Motor 3
    drone(8) = surface(xcylinder, ycylinder - b/2, H_m * zcylinder + H/2, 'facecolor', 'r');  % Motor 4
    alpha(drone(5:8), 0.7);  % Set transparency

    drone(9)  = patch(xp + b/2, yp, zp + (H_m + H/2), 'c', 'LineWidth', 0.5);  % Propeller 1
    drone(10) = patch(xp - b/2, yp, zp + (H_m + H/2), 'c', 'LineWidth', 0.5);  % Propeller 2
    drone(11) = patch(xp, yp + b/2, zp + (H_m + H/2), 'p', 'LineWidth', 0.5);  % Propeller 3
    drone(12) = patch(xp, yp - b/2, zp + (H_m + H/2), 'p', 'LineWidth', 0.5);  % Propeller 4
    alpha(drone(9:12), 0.3);  % Set transparency

    % Create a group object and parent surface
    combinedobject = hgtransform('parent', hg);  % Create a transform object for combined movement
    set(drone, 'parent', combinedobject);  % Set drone parts as children of the transform object

    % Animation loop
    for i = 1:length(xe)
        % Plot the trajectory
        plot3(xe(1:i), ye(1:i), h(1:i), 'b:', 'LineWidth', 1.5);  

        % Define the transformation for current position and orientation
        translation = makehgtform('translate', [xe(i) ye(i) h(i)]);
        rotation1 = makehgtform('xrotate', phi(i));
        rotation2 = makehgtform('yrotate', theta(i));
        rotation3 = makehgtform('zrotate', psi(i));

        % Combine transformations and apply to the drone
        set(combinedobject, 'matrix', translation * rotation3 * rotation2 * rotation1);

        drawnow;  % Update the figure
        pause(0.02);  % Pause for a short time to control animation speed
    end
end
