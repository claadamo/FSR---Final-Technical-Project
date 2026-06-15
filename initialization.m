%% INITIALIZATION: AUV PARAMETERS & GEOMETRIC STATIC ALLOCATION

% --- Rigid Body & Hydrodynamic Added Mass Properties ---
% Hydrodynamic added mass (linear components) mapping directional water resistance
M_a = diag([3.7716, 12.7006, 16.4712]);
M = 18.5; % Vehicle dry mass in kg
mass = M * eye(3) + M_a; % Combined virtual mass tensor matrix (Rigid body + Added mass)

% Rigid body and added mass inertia tensor matrices mapping rotational resistance
Ib_added = diag([0.0545, 1.2628, 0.8198]);
Ib = diag([0.3432, 1.4483, 1.5047]) + Ib_added; % Total inertia tensor matrix

% --- Environmental & Hydrostatic Parameters ---
g = 9.81; % Gravitational acceleration (m/s^2)
e3 = [0;0;1]; % Gravity unit vector (Z-down convention, pointing to seabed)
rho = 1000; % Fluid density of water (kg/m^3)
Delta = M / rho; % Displaced volume in m^3 computed from virtual mass

% --- Propeller Thrust Properties (BlueRobotics T200) ---
% Quadratic lumped thrust coefficients mapping rotational speed (rad/s) to force (N)
cf_forward  = 4.7e-5;  % Nominal coefficient when pushing forward
cf_backward = 2.9e-5;  % Reduced coefficient when pushing backward due to blade asymmetry

% --- Hydrodynamic Damping Coefficients ---
% Linear damping diagonal matrix modeling low-speed skin friction
D_L = diag([5.9341, 4.3172, 6.2491, 0.0199, 0.4611, 0.0698]);
% Quadratic damping diagonal matrix modeling high-speed profile drag
D_Q = diag([19.374, 42.687, 51.0402, 0.0668, 1.9172, 1.1343]);

% --- Hydrostatic Offsets (Body Frame Vectors) ---
r_c_b = [0; 0; 0];      % Center of Mass (CoM) position vector
r_b_b = [-0.2; 0; 0.2]; % Center of Buoyancy (CoB) position vector creating restoring torques

%% SYSTEM SAMPLING TIME & INPUT SATURATION BOUNDS
Ts = 0.001; % Fixed sampling time for numerical integration solver (1 ms)
% Actuator thrust limits (Saturation bounds in Newtons per thruster channel)
max_value_sat = 114; % Threshold boundary

%% OPENAUV FRAME GEOMETRY: THRUSTER PLACEMENT & DIRECTION MATRIX
% Calibrated structural lever arms from vehicle center of geometry (in meters)
dx_horiz = 0.700/2;     % Longitudinal distance for horizontal engines (700/2 mm)
dy_horiz = 0.280/2;     % Transverse distance for horizontal engines (280/2 mm)
dx_vert  = 0.470/2;     % Longitudinal distance for vertical engines (470/2 mm)
dy_vert  = 0.260/2;     % Transverse distance for vertical engines (260/2 mm)

% 1. Position Vectors Allocation (r_i = [x; y; z] relative to Body COG)
r = zeros(3, 8);
r(:,1) = [ dx_horiz;  dy_horiz; 0]; % M1: Front-Right Horizontal
r(:,2) = [ dx_horiz; -dy_horiz; 0]; % M2: Front-Left Horizontal
r(:,3) = [-dx_horiz;  dy_horiz; 0]; % M3: Rear-Right Horizontal
r(:,4) = [-dx_horiz; -dy_horiz; 0]; % M4: Rear-Left Horizontal
r(:,5) = [ dx_vert;  dy_vert; 0];   % M5: Front-Right Vertical
r(:,6) = [ dx_vert; -dy_vert; 0];   % M6: Front-Left Vertical
r(:,7) = [-dx_vert;  dy_vert; 0];   % M7: Rear-Right Vertical
r(:,8) = [-dx_vert; -dy_vert; 0];   % M8: Rear-Left Vertical

% Decomposition factor for the 45-degree fixed canting layout
inc = cos(pi/4); % 0.7071 (equal components along X and Y body axes)

% 2. Unit Thrust Vectors Allocation (d_i = [dx; dy; dz] fixed mapping)
d = zeros(3, 8);
d(:,1) = [ inc;  inc;  0];  % M1: Thrusts forward and leftward
d(:,2) = [ inc;  -inc;  0]; % M2: Thrusts forward and rightward
d(:,3) = [-inc;  inc;  0];  % M3: Thrusts backward and leftward
d(:,4) = [-inc;  -inc;  0]; % M4: Thrusts backward and rightward
d(:,5) = [ 0;  0;  1];      % M5: Vertical thruster pointing Downward (+Z)
d(:,6) = [ 0;  0; -1];      % M6: Vertical thruster pointing Upward (-Z)
d(:,7) = [ 0;  0; -1];      % M7: Vertical thruster pointing Upward (-Z)
d(:,8) = [ 0;  0;  1];      % M8: Vertical thruster pointing Downward (+Z)

% 3. Generation of Static Thruster Configuration Matrix B (6x8)
B = zeros(6, 8);
for i = 1:8
    B(1:3, i) = d(:, i);                 % Top 3 rows: force projection components
    B(4:6, i) = cross(r(:, i), d(:, i)); % Bottom 3 rows: induced moment leverage arms
end
B(abs(B) < 1e-10) = 0; % Eliminate floating-point truncation noise

%% TRAJECTORY GENERATION PLANNER (7th-ORDER MINIMUM JERK POLYNOMIAL)
% --- Time Domain Definition ---
t_iniz = 0;    
ttot = 22; % [s] slow trajectory
% ttot = 8; % [s] fast trajectory
tdead = 10;    
tot_time = ttot + tdead;

t1 = t_iniz : Ts : ttot;                   
t_dead_vec = (ttot + Ts) : Ts : tot_time;  
t = [t1, t_dead_vec];                      

% Memory allocation arrays
p       = zeros(4, length(t1));     
dot_p   = zeros(4, length(t1));     
ddot_p  = zeros(4, length(t1));
dddot_p = zeros(4, length(t1));

p_d       = zeros(4, length(t));   
dot_p_d   = zeros(4, length(t));   
ddot_p_d  = zeros(4, length(t));   
dddot_p_d = zeros(4, length(t));

% --- Boundary Conditions
% slow trajectory
x0 = 0;  xf = 1; % [m]
y0 = 0;  yf = 1; % [m]
z0 = -1; zf = -4; % [m]
% % fast trajectory
% x0 = 0;  xf = 5; % [m]
% y0 = 0;  yf = 5; % [m]
% z0 = -1; zf = -6; % [m]

dot_x0 = 0; dot_xf = 0; ddot_x0 = 0; ddot_xf = 0; dddot_x0 = 0; dddot_xf = 0;
dot_y0 = 0; dot_yf = 0; ddot_y0 = 0; ddot_yf = 0; dddot_y0 = 0; dddot_yf = 0;
dot_z0 = 0; dot_zf = 0; ddot_z0 = 0; ddot_zf = 0; dddot_z0 = 0; dddot_zf = 0;

Rb0 = eye(3); 
roll_f = pi/4; pitch_f = pi/4; yaw_f = pi; 

% Compute desired target orientation matrix using ZYX sequence mapping
Rf = [cos(pitch_f)*cos(yaw_f), sin(roll_f)*sin(pitch_f)*cos(yaw_f)-cos(roll_f)*sin(yaw_f), cos(roll_f)*sin(pitch_f)*cos(yaw_f)+sin(roll_f)*sin(yaw_f);
      cos(pitch_f)*sin(yaw_f), sin(roll_f)*sin(pitch_f)*sin(yaw_f)+cos(roll_f)*cos(yaw_f), cos(roll_f)*sin(pitch_f)*sin(yaw_f)-sin(roll_f)*cos(yaw_f);
      -sin(pitch_f),           sin(roll_f)*cos(pitch_f),                                 cos(roll_f)*cos(pitch_f)];
  
Rbf = Rb0'*Rf; 
theta_f = acos(0.5*(Rbf(1,1)+Rbf(2,2)+Rbf(3,3)-1)); 
axis_r = (1/2/sin(theta_f))*[Rbf(3,2)-Rbf(2,3); Rbf(1,3)-Rbf(3,1); Rbf(2,1)-Rbf(1,2)]; 
theta_0 = 0; 

% Grouping boundary states
p0 = [x0, y0, z0, theta_0]; dot_p0 = [dot_x0, dot_y0, dot_z0, 0]; ddot_p0 = [ddot_x0, ddot_y0, ddot_z0, 0]; dddot_p0 = [dddot_x0, dddot_y0, dddot_z0, 0];
pf = [xf, yf, zf, theta_f]; dot_pf = [dot_xf, dot_yf, dot_zf, 0]; ddot_pf = [ddot_xf, ddot_yf, ddot_zf, 0]; dddot_pf = [dddot_xf, dddot_yf, dddot_zf, 0];

% --- 7th-Order Polynomial Solver & Interpolation Loop ---
a0=zeros(1,4); a1=zeros(1,4); a2=zeros(1,4); a3=zeros(1,4); a4=zeros(1,4); a5=zeros(1,4); a6=zeros(1,4); a7=zeros(1,4);
for j=1:4
    A = [t_iniz^7, t_iniz^6, t_iniz^5, t_iniz^4, t_iniz^3, t_iniz^2, t_iniz, 1;
        ttot^7, ttot^6, ttot^5, ttot^4, ttot^3, ttot^2, ttot, 1;
        7*t_iniz^6, 6*t_iniz^5, 5*t_iniz^4, 4*t_iniz^3, 3*t_iniz^2, 2*t_iniz, 1, 0;
        7*ttot^6, 6*ttot^5, 5*ttot^4, 4*ttot^3, 3*ttot^2, 2*ttot, 1, 0;
        42*t_iniz^5, 30*t_iniz^4, 20*t_iniz^3, 12*t_iniz^2, 6*t_iniz, 2, 0, 0;
        42*ttot^5, 30*ttot^4, 20*ttot^3, 12*ttot^2, 6*ttot, 2, 0, 0;
        210*t_iniz^4, 120*t_iniz^3, 60*t_iniz^2, 24*t_iniz, 6, 0, 0, 0;
        210*ttot^4, 120*ttot^3, 60*ttot^2, 24*ttot, 6, 0, 0, 0];
    
    b = [p0(j) pf(j) dot_p0(j) dot_pf(j) ddot_p0(j) ddot_pf(j) dddot_p0(j) dddot_pf(j)]';
    
    a_temp = A\b;
    a7(j) = a_temp(1); a6(j) = a_temp(2); a5(j) = a_temp(3); a4(j) = a_temp(4);
    a3(j) = a_temp(5); a2(j) = a_temp(6); a1(j) = a_temp(7); a0(j) = a_temp(8);
    
    p(j,:)     = a7(j)*t1.^7 + a6(j)*t1.^6 + a5(j)*t1.^5 + a4(j)*t1.^4 + a3(j)*t1.^3 + a2(j)*t1.^2 + a1(j)*t1 + a0(j);
    dot_p(j,:) = 7*a7(j)*t1.^6 + 6*a6(j)*t1.^5 + 5*a5(j)*t1.^4 + 4*a4(j)*t1.^3 + 3*a3(j)*t1.^2 + 2*a2(j)*t1 + a1(j);
    ddot_p(j,:)= 42*a7(j)*t1.^5 + 30*a6(j)*t1.^4 + 20*a5(j)*t1.^3 + 12*a4(j)*t1.^2 + 6*a3(j)*t1 + 2*a2(j);
    dddot_p(j,:)= 210*a7(j)*t1.^4 + 120*a6(j)*t1.^3 + 60*a5(j)*t1.^2 + 24*a4(j)*t1 + 6*a3(j);
    
    p_d(j,:)       = [p(j,:),     zeros(1, length(t_dead_vec)) + pf(j)]; 
    dot_p_d(j,:)   = [dot_p(j,:), zeros(1, length(t_dead_vec)) + dot_pf(j)];
    ddot_p_d(j,:)  = [ddot_p(j,:), zeros(1, length(t_dead_vec)) + ddot_pf(j)];
    dddot_p_d(j,:) = [dddot_p(j,:),zeros(1, length(t_dead_vec)) + dddot_pf(j)];
end

% --- Axis-Angle Transformation to Rotational Kinematics ---
N_time = length(t); 
Rbdes = zeros(3, 3, N_time);          
omegabb_des = zeros(3, N_time);      
dot_omegabb_des = zeros(3, N_time);  

for i = 1:N_time
    tetai = p_d(4,i);
    dot_tetai = dot_p_d(4,i);
    ddot_tetai = ddot_p_d(4,i);
    temp1 = 1 - cos(tetai);
    
    Ri = [axis_r(1)^2*temp1+cos(tetai),           axis_r(1)*axis_r(2)*temp1-axis_r(3)*sin(tetai), axis_r(1)*axis_r(3)*temp1+axis_r(2)*sin(tetai);
          axis_r(1)*axis_r(2)*temp1+axis_r(3)*sin(tetai), axis_r(2)^2*temp1+cos(tetai),           axis_r(2)*axis_r(3)*temp1-axis_r(1)*sin(tetai);
          axis_r(1)*axis_r(3)*temp1-axis_r(2)*sin(tetai), axis_r(2)*axis_r(3)*temp1+axis_r(1)*sin(tetai), axis_r(3)^2*temp1+cos(tetai)];
    
    Rbdes(:,:,i) = Rb0*Ri;
    omegabb_des(:,i) = Rbdes(:,:,i)'*Rb0*axis_r*dot_tetai;      
    dot_omegabb_des(:,i) = Rbdes(:,:,i)'*Rb0*axis_r*ddot_tetai;  
end

%% SIMULINK COMPATIBILITY BUS INITIALIZATION
pos_0 = [x0 y0 z0];
lin_vel_0 = [dot_x0 dot_y0 dot_z0];
w_bb_0 = [0 0 0];

csi_d = p_d(1:3,:); dot_csi_d = dot_p_d(1:3,:); ddot_csi_d = ddot_p_d(1:3,:); dddot_csi_d = dddot_p_d(1:3,:);
psi_d = p_d(4,:);   dot_psi_d = dot_p_d(4,:);   ddot_psi_d = ddot_p_d(4,:);   dddot_psi_d = dddot_p_d(4,:);