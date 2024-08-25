function xdot = drone_eom(t, x, parm, controls, demands)
% DRONE_EOM - Equations of Motion for a Drone
% This function calculates the derivatives of various states for a drone
% given inputs such as controls and demands.
%
% Inputs:
%   x - state vector [u, v, w, p, q, r, phi, theta, psi, xe, ye, h]'
%   parm - parameter vector [W, Ixx, Iyy, Izz, Ixz, S, cbar, span, CT, CY, CZ, Cm, Cl, Cn, Ieng, weng]
%          W - total weight (N)
%          Ixx - moment of inertia about x-axis (kg*m^2)
%          Iyy - moment of inertia about y-axis (kg*m^2)
%          Izz - moment of inertia about z-axis (kg*m^2)
%          Ixz - product of inertia (kg*m^2)
%          S - reference area (m^2)
%          cbar - distance from center of gravity to reference point (m)
%          span - wing span (m)
%          CT - thrust coefficient
%          CY - side force coefficient
%          CZ - vertical force coefficient
%          Cm - pitching moment coefficient
%          Cl - roll moment coefficient
%          Cn - yawing moment coefficient
%          Ieng - engine moment of inertia (kg*m^2)
%          weng - engine angular velocity (rad/s)
%   controls - control inputs [T1, T2, T3, T4]
%   demands - demanded states [hd, ud]
%            hd - demanded height (m)
%            ud - demanded velocity (m/s)
%
% Outputs:
%   xdot: derivatives of the state vector

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parameters
g = 9.81; % gravitational acceleration (m/s^2)
W = parm(1); % total weight (N)
Ixx = parm(2); % moment of inertia about x-axis (kg*m^2)
Iyy = parm(3); % moment of inertia about y-axis (kg*m^2)
Izz = parm(4); % moment of inertia about z-axis (kg*m^2)
Ixz = parm(5); % product of inertia (kg*m^2)
%S = parm(6); % reference area (m^2) - commented out
cbar = parm(7); % distance from center of gravity to reference point (m)
%span = parm(8); % wing span (m) - commented out
CT = parm(9); % thrust coefficient
CY = parm(10); % side force coefficient
%CZ = parm(11); % vertical force coefficient - commented out
Ax = parm(12); % pitching moment coefficient - commented out
Ay = parm(13); % roll moment coefficient - commented out
Az = parm(14); % yawing moment coefficient - commented out
Ieng = parm(15); % moment of inertia of engine
weng = parm(16); % engine angular velocity

% State Variables
u = x(1);  % velocity along x-axis (m/s)
v = x(2);  % velocity along y-axis (m/s)
w = x(3);  % velocity along z-axis (m/s)
p = x(4);  % roll rate (rad/s)
q = x(5);  % pitch rate (rad/s)
r = x(6);  % yaw rate (rad/s)
phi = x(7); % roll angle (rad)
theta = x(8); % pitch angle (rad)
psi = x(9); % yaw angle (rad)
xe = x(10); % east position (m)
ye = x(11); % north position (m)
h = x(12); % altitude (m)

% Controls
T1 = controls(1); % thrust 1 (Newtons)
T2 = controls(2); % thrust 2 (Newtons)
T3 = controls(3); % thrust 3 (Newtons)
T4 = controls(4); % thrust 4 (Newtons)

% Total vertical thrust
Fz = T1 + T2 + T3 + T4; % Total vertical thrust (Newtons)
% Fz=Fz+g * cos(theta) * cos(phi); %  cancel the weight component
%size(Klqr)
%size(x)

% Torques
L = cbar; % arm length (meters)
k = CT; % thrust coefficient
b = CY; % drag coefficient
tau = [L * k * (T1 - T3);  L * k * (T2 - T4);   b * (T1 - T2 + T3 - T4)]; % torques (N*m)

% Demands
hd = demands(1); % demanded height (m)
ud = demands(2); % demanded velocity (m/s)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Intermediate Calculations
mass = W / g; % mass of the drone (kg)
%S = Spare; % reference area (m^2)
%cbar = L; % distance from center of gravity to reference point (m)
%span = 1; % wing span (m)
%CT = k; % thrust coefficient
%CY = 1; % side force coefficient
%CZ = 1; % vertical force coefficient
%Cm = 1; % pitching moment coefficient
%Cl = 1; % roll moment coefficient
%Cn = 1; % yawing moment coefficient

% Calculating auxiliary variables
heng = Ieng * weng; % engine angular momentum
%qd = 0.5 * W / g * ud^2; % dynamic pressure (Pa)
%qds = qd*S/mass;  % dynamic pressure dynamic pressure per unit mass (Pa/kg)

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

% Equations of motion
udot = -Ax*u+r * v - q * w - g * sin(theta); %  + qds * (CX + CT); % Acceleration in the x-direction
vdot = -Ay*v+p * w - r * u + g * cos(theta) * sin(phi);%  + qds * CY; % Acceleration in the y-direction
wdot = -Az*w-q * u - p * v + g * cos(theta) * cos(phi) - Fz;% + qds * CZ; % Acceleration in the z-direction
pdot = (c1 * r + c2 * p + c4 * heng) * q +  (c3 * tau(1) + c4 * tau(3));% + qd * S * span * (c3 * Cl + c4 * Cn); % Roll rate derivative
qdot = (c5 * p - c7 * heng) * r - c6 * (p * p - r * r) + c7 * tau(2);% + qd * S * cbar * c7 * Cm; % Pitch rate derivative
rdot = (c8 * p - c2 * r + c9 * heng) * q + c4 * tau(1) + c9 * tau(3);% + qd * S * span * (c4 * Cl + c9 * Cn); % Yaw rate derivative
phi_dot = p + tan(theta) * (q * sin(phi) + r * cos(phi)); % Roll angle derivative
theta_dot = q * cos(phi) - r * sin(phi); % Pitch angle derivative
psi_dot = (q * sin(phi) + r * cos(phi)) / cos(theta); % Yaw angle derivative
xe_dot = u * cos(psi) * cos(theta) + v * (cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi)) ...
    + w * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)); % East position derivative
ye_dot = u * sin(psi) * cos(theta) + v * (sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi)) ...
    + w * (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)); % North position derivative
hdot = u * sin(theta) - v * cos(theta) * sin(phi) - w * cos(theta) * cos(phi); % Height rate derivative

% State Derivatives
xdot = [udot; vdot; wdot; pdot; qdot; rdot; phi_dot; theta_dot; psi_dot; xe_dot; ye_dot; hdot];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
