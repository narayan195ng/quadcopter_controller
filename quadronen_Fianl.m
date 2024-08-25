% Quadrone.m
clear all % Clear all variables from memory
close all % Close all open figures
%clc       % Clear the command window
clear     % Clear workspace
%clearvars % Clear workspace variables

% Define constants
pi = atan(1)*4; % Define pi
piby6 = pi/6;   % Define pi/6
rads = pi/180;  % Conversion factor from degrees to radians

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters
W = 9.81 * 2.5;  % Total weight (N) - Total weight of the aircraft in Newtons
Ixx = 4.856e-3;  % Moment of inertia about x-axis (kg*m^2) - Moment of inertia about the x-axis in kg*m^2
Iyy = 4.856e-3;  % Moment of inertia about y-axis (kg*m^2) - Moment of inertia about the y-axis in kg*m^2
Izz = 8.801e-3;  % Moment of inertia about z-axis (kg*m^2) - Moment of inertia about the z-axis in kg*m^2
Ixz = 0;  % Product of inertia (kg*m^2) - Product of inertia in kg*m^2
cbar = 0.1;  % Distance from center of gravity to reference point (m) - Distance from the center of gravity to the reference point in meters
CT = 0.1;  % Thrust coefficient - Thrust coefficient
CY = 0.1;  % Side force coefficient - Side force coefficient
Ieng = 0.05;  % Moment of inertia of engine - Moment of inertia of the engine
weng = 0;  % Engine angular velocity - Engine angular velocity
Az = 0.25;  % Aerodynamic force in z-direction
Ay = 0.25;  % Aerodynamic force in y-direction
Ax = 0.25;  % Aerodynamic force in x-direction
L=cbar; 
k=CT; 
b=CY;

% Controls
T0=2.5;
T1 = T0;  % Thrust 1 (Newtons) - Thrust applied by control 1 in Newtons
T2 = T0;  % Thrust 2 (Newtons) - Thrust applied by control 2 in Newtons
T3 = T0;  % Thrust 3 (Newtons) - Thrust applied by control 3 in Newtons
T4 = T0;  % Thrust 4 (Newtons) - Thrust applied by control 4 in Newtons

% Demands
hd = 15;  % Demanded height (m) - Demanded height in meters
ud = 10;  % Demanded velocity (m/s) - Demanded velocity in meters per second
thetad=pi/12;

% Parameters (updating)
parm(1) = W; % total weight (N) - Update total weight parameter
parm(2) = Ixx; % moment of inertia about x-axis (kg*m^2) - Update moment of inertia about x-axis parameter
parm(3) = Iyy; % moment of inertia about y-axis (kg*m^2) - Update moment of inertia about y-axis parameter
parm(4) = Izz; % moment of inertia about z-axis (kg*m^2) - Update moment of inertia about z-axis parameter
parm(5) = Ixz; % product of inertia (kg*m^2) - Update product of inertia parameter
parm(7) = cbar; % distance from center of gravity to reference point (m) - Update distance from center of gravity to reference point parameter
parm(9) = CT; % thrust coefficient - Update thrust coefficient parameter
parm(10) = CY; % side force coefficient - Update side force coefficient parameter
parm(12) = Ax; % pitching moment coefficient - Update pitching moment coefficient parameter
parm(13) = Ay; % roll moment coefficient - Update roll moment coefficient parameter
parm(14) = Az; % yawing moment coefficient - Update yawing moment coefficient parameter
parm(15) = Ieng; % moment of inertia of engine - Update moment of inertia of engine parameter
parm(16) = weng; % engine angular velocity - Update engine angular velocity parameter

% Controls (updating)
controls(1) = T1; % thrust 1 (Newtons) - Update thrust 1 parameter
controls(2) = T2; % thrust 2 (Newtons) - Update thrust 2 parameter
controls(3) = T3; % thrust 3 (Newtons) - Update thrust 3 parameter
controls(4) = T4; % thrust 4 (Newtons) - Update thrust 4 parameter
Klqr=zeros(4,12);

% Demands (updating)
demands(1) = hd; % demanded height (m) - Update demanded height parameter
demands(2) = ud; % demanded velocity (m/s) - Update demanded velocity parameter
demands(3) =-thetad; % Must tilt the drone forward to go forward

% Loop over control modes
for icon = 0:1
    % Check the control mode
    if icon == 0
        % Uncontrolled Drone
        disp('Uncontrolled Drone');
        
    elseif icon == 1
        % PID Controlled Drone
        disp('PID Controlled Drone');
        % Run PID Control Simulation

        % Define PID Controller Parameters for velocity control
        Kp_u = 1.0; % Proportional gain for velocity control
        Ki_u = 0.5; % Integral gain for velocity control
        Kd_u = 2.0; % Derivative gain for velocity control

        % Define PID Controller Parameters for height control
        Kp_h = 2.0; % Proportional gain for height control
        Ki_h = 1.0; % Integral gain for height control
        Kd_h = 0.5; % Derivative gain for height control

        % Initialize PID Controller states for velocity control
        integral_u = 0; % Integral term initialization for velocity control
        previous_error_u = 0; % Previous error initialization for velocity control

        % Initialize PID Controller states for height control
        integral_h = 0; % Integral term initialization for height control
        previous_error_h = 0; % Previous error initialization for height control

    elseif icon == 2
        % LQR Controlled Drone
        disp('LQR Controlled Drone');

        % Define gravitational constant and engine angular momentum
        g = 9.81; % Gravitational constant (m/s^2)
        heng = Ieng * weng; % Engine angular momentum

        % Moment of inertia determinant
        Idel = Ixx * Izz - Ixz * Ixz; % Moment of inertia determinant (kg*m^2)

        % Moments of Inertia Calculations
        c1 = ((Iyy - Izz) * Izz - Ixz^2) / Idel;
        c2 = ((Ixx - Iyy + Izz) * Ixz) / Idel;
        c3 = Izz / Idel;
        c4 = Ixz / Idel;
        c5 = (Izz - Ixx) / Iyy;
        c6 = Ixz / Iyy;
        c7 = 1 / Iyy;
        c8 = ((Ixx - Iyy) * Ixx - Ixz^2) / Idel;
        c9 = Ixx / Idel;

        % State-space representation (for demonstration purposes, need actual system matrices)
        A = [-Az 0 0 0 0 0 0 0;
             0 -0.0001 c4*heng 0 0 0 0 0;
             0 0 -0.0001 -c7*heng 0 0 0 0;
             0 0 c9*heng -0.0001 0 0 0 0;
             0 1 0 0 0 0 0 0;
             0 0 1 0 0 0 0 0;
             0 0 0 1 0 0 0 0;
             -1 0 0 0 0 0 0 0];

        % Controls are T1, T2, T3, T4 (NOT Fz, tau1, tau2 and tau3)
        B1 = [-1 0 0 0;
              0 c3 0 c4;
              0 0 c7 0;
              0 c4 0 c9];

        % Transform to T1, T2, T3, T4
        B1 = B1 * [1 1 1 1; L*k 0 -L*k 0; 0 L*k 0 -L*k; b -b b -b];
        B = [B1; 0*eye(4)];

        % Check controllability
        QC = rank(ctrb(A,B)); % Rank of controllability matrix

        % Output matrix C and direct transmission matrix D
        C = eye(8);
        D = 0;

        % Weighting matrices for LQR
        Q = eye(8);
        R = 0.8 * eye(4);

        % Compute LQR gain
        Klqr = lqr(A, B, Q, R);

        % Initialize PID Controller parameters for the LQR control
        integral_u = 0; % Integral term initialization for velocity control
        previous_error_u = 0; % Previous error initialization for velocity control
        integral_h = 0; % Integral term initialization for height control
        previous_error_h = 0; % Previous error initialization for height control
     end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Initialize variables
    yi = zeros(12,1);     % Initial condition for position and velocity
    % h = 100; % Altitude
    yi = [ud; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; hd]; % Initial state vector
    % yi = [u; v; w; p; q; r; phi; theta; psi; xe; ye; h];
    
    % parm = [W, Ixx, Iyy, Izz, Ixz, cbar, CT, CY, Ieng, weng]; % Drone parameters
    % demands = [hd, ud];      % Initialize demands with values
    % controls = [T1, T2, T3, T4];  % Initialize controls with values
    
    % Initialize state variable and history
    s = yi; % Initialize state variable
    xV = []; % Initialize state history
    time = []; % Initialize time history
    time = [time 0]; % Store initial time
    xV = [xV yi]; % Store initial state vector
    
    % Simulation parameters
    N = 100000; % Number of iterations 5 bole toh 10
    t0 = 0; % Initial time
    dt = 0.0001; % Time step size
    tt = t0; % Initialize time variable
    
    
    % Preallocate storage for time and state vectors
    time = zeros(1, N);
    xV = zeros(12, N);
    
    % Calculate parameters related to atmospheric conditions
    [~, aat, pat, rho_sl, nu_sl] = atmosisa(0); % Standard atmospheric conditions at sea level
    [Tat, aat, pat, rho_h, nu_h] = atmosisa(yi(12)); % Atmospheric conditions at altitude h
    
    % Update parameters 9 and 10
    parm(9) = parm(7) * rho_h / rho_sl; % Adjusted thrust coefficient
    parm(10) = parm(8) * rho_h / rho_sl; % Adjusted drag coefficient
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Main loop
        for k = 1:N % Loop from 1 to N iterations

            % Calculate kstar (modulo operation)
            kstar1 = k - 1000000 * floor(k/10000); % Find remainder after dividing k by 10000
            
            % Check if kstar is 0 (every 10000th step)
            if kstar1 == 0
                % Store current time and state
                time(k) = k * dt; % Store time
                xV(:,k) = s; % Store state vector
                
            end

            if icon == 1
                % Define Controller parameters
                % Compute the errors
                error_h = demands(1) - s(12); % Calculate the error in height (desired height - current height)
                error_u = demands(2) - s(2); % Calculate the error in velocity (desired velocity - current velocity)

                % Update the integral terms
                integral_h = integral_h + error_h * dt; % Update integral of height error
                integral_u = integral_u + error_u * dt; % Update integral of velocity error

                % Compute the derivative terms
                derivative_h = (error_h - previous_error_h) / dt; % Calculate derivative of height error
                derivative_u = (error_u - previous_error_u) / dt; % Calculate derivative of velocity error

                % Update previous errors
                previous_error_h = error_h; % Store current height error for next iteration
                previous_error_u = error_u; % Store current velocity error for next iteration

                % Compute the PID outputs
                pid_output_h = Kp_h * error_h + Ki_h * integral_h + Kd_h * derivative_h; % Compute PID control output for height
                pid_output_u = Kp_u * error_u + Ki_u * integral_u + Kd_u * derivative_u; % Compute PID control output for velocity

                % Adjust thrusts based on PID output
                T0 = 2.5 + pid_output_h; % Base thrust adjusted by PID for height control
                T1 = T0;  % Set thrust 1 to T0 (Newtons)
                T2 = T0;  % Set thrust 2 to T0 (Newtons)
                T3 = T0;  % Set thrust 3 to T0 (Newtons)
                T4 = T0;  % Set thrust 4 to T0 (Newtons)

                % Update controls
                controls(1) = T1; % Update control for thrust 1
                controls(2) = T2; % Update control for thrust 2
                controls(3) = T3; % Update control for thrust 3
                controls(4) = T4; % Update control for thrust 4

            elseif icon == 2 
                % LQR CONTROL
                % Define the desired states (setpoints)
                desired_state = [demands(2); 0; 0; 0; 0; 0; 0; demands(3); 0; 0; 0; demands(1)];

                % Calculate the state error
                e = desired_state - s;
                reduced_error_state=[e(3) e(4) e(5) e(6) e(7) e(8) e(9) e(12)]';
                % Compute the LQR control input
                ulqr = Klqr * reduced_error_state;

                % Update controls based on LQR outputs
                controls(1) = T0 + ulqr(1); % Thrust 1
                controls(2) = T0 + ulqr(2); % Thrust 2
                controls(3) = T0 + ulqr(3); % Thrust 3
                controls(4) = T0 + ulqr(4); % Thrust 4
             end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Simulate process and measurement
            tspan = [(k - 1) * dt, k * dt]; % Define the time span for the simulation (current step duration)
            options = odeset('RelTol', 1e-6, 'AbsTol', 1e-7); % Set options for ODE solver with specific tolerances
            
            % Solve ODE system
            [tfin, yfin] = ode45(@(t,y)drone_eom(t, y, parm, controls, demands), tspan, s, options); % Solve the ODEs for the drone's equations of motion
           
            % Extract dimensions of output
            [nt, nj] = size(yfin); % Get size of the final state matrix
        
            % Extract final state
            s = yfin(end, :)'; % Update the state vector with the final state from the ODE solver
         
            % Extract final state
            s0 = yfin(nt, :)'; % Get the final state
            
            % Update process for the next step
            s = s0; % Update state vector
            
            % Update atmospheric properties
            h = s(12);
            [Tat, aat, pat, rho_h, nu_h] = atmosisa(h);
            parm(9) = CT * rho_h / rho_sl;
            parm(10) = CY * rho_h / rho_sl;
            
            % Update time
            tt = tt + dt; % Increment time by time step

            % Store time and state every 10 steps
            if mod(k, 10) == 0 % Check if the current step is a multiple of 10
                time(k/10) = k * dt; % Store the current time
                xV(:,k/10) = s; % Store the current state vector
            end  
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %   % Store time and state every 10000 steps
        %     kstar1 = k - 10000 * floor(k / 10000); % Find remainder after dividing k by 10000
        %     if kstar1 == 0 % Check if the current step is a multiple of 10000
        %         time(k / 10000) = k * dt; % Store the current time
        %         xV(:, k / 10000) = s; % Store the current state vector
        %     end
        % end
        % 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Trim unused preallocated space
time = time(1:floor(N/10)); % Resize the time array to include only the used elements
xV = xV(:, 1:floor(N/10)); % Resize the state vector array to include only the used elements

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% size(time) % Display size of time vector
% size(xV) % Display size of state vector matrix

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Plot results
figure(1+icon) % Create a new figure window for plotting

   for k = 1:6 % Loop through each subplot
    subplot(6,1,k) % Divide the figure window into 6 subplots vertically

        if k == 1
            plot(time, xV(1,:), '-m', 'lineWidth', 3) % Plot forward velocity vs. time with magenta color and thicker line
            title('Drone Response plots') % Set the title for the entire set of subplots
            ylabel('Fwd u') % Label the y-axis for the first subplot as "Fwd u"
        elseif k == 2
            plot(time, xV(2,:), '-g', 'lineWidth', 3) % Plot vertical velocity v vs. time with green color and thicker line
            ylabel('Vert. v') % Label the y-axis for the third subplot as "Vert. v"
        elseif k == 3
            plot(time, xV(3,:), '-r', 'lineWidth', 3) % Plot vertical velocity vs. time with red color and thicker line
            ylabel('Vert. w') % Label the y-axis for the second subplot as "Vert. w"
        elseif k == 4
            plot(time, xV(4,:), '-b', 'lineWidth', 3) % Plot roll rate vs. time with blue color and thicker line
            ylabel('Roll Rate') % Label the y-axis for the fourth subplot as "Roll Rate"
        elseif k == 5
            plot(time, xV(5,:), '-b', 'lineWidth', 3) % Plot pitch rate vs. time with blue color and thicker line
            ylabel('Pitch Rate') % Label the y-axis for the fifth subplot as "Pitch Rate"
        elseif k == 6
            plot(time, xV(12,:), '-m', 'lineWidth', 3) % Plot height vs. time with magenta color and thicker line
            ylabel('Height') % Label the y-axis for the sixth subplot as "Height"
        end

        xlabel('Time in seconds') % Label the x-axis for all subplots as "Time in seconds"
        grid on % Display grid lines on the plot
   end
end


% % Now, create a completely new figure for the animation
% figure(2); % Create a new figure window for the animation
% disp('Animating PID and LQR control responses...');
% 
% % Extract required state variables for animation
% xe = xV(10, :);
% ye = xV(11, :);
% h = xV(12, :);
% phi = xV(7, :);
% theta = xV(8, :);
% psi = xV(9, :);
% 
% % Perform the animation in the new figure
% drone_Animation(xe, ye, h, phi, theta, psi);

function [Tat, aat, pat, rho_sl, nu_sl] = atmosisa(h)
    % Constants
    R = 287; % Specific gas constant for dry air in J/(kg*K)
    g = 9.81; % Acceleration due to gravity in m/s^2
    P0 = 101325; % Standard atmospheric pressure at sea level in Pa
    T0 = 288.15; % Standard temperature at sea level in K
    L = 0.0065; % Standard temperature lapse rate in K/m
    gamma = 1.4; % Specific heat ratio for air (Cp/Cv)
    
    % Calculate atmospheric properties
    Tat = T0 - L * h; % Temperature at altitude h (in meters) using linear lapse rate
    aat = sqrt(gamma * R * Tat); % Speed of sound at altitude h (using Tat)
    pat = P0 * (1 - L * h / T0)^(g / (R * L)); % Pressure at altitude h (using barometric formula)
    rho_sl = pat / (R * Tat); % Density at altitude h (using ideal gas law)
    nu_sl = 1.461 * 10^-5 * sqrt(Tat); % Kinematic viscosity at altitude h (using Sutherland's formula)
end
