%% ========================================================================
% MASTER INITIALIZATION SCRIPT: M04 LQR Optimal Control + Smart Throttle
% Architecture: Linear Quadratic Regulator (Lateral) + Smart Throttle (Longitudinal)
% Purpose: The "Final Boss" of the Active Stability study. Replaces the reactive 
% P-controller with a predictive, optimal state-space controller. By balancing 
% Cross-Track Error (e_cg) and Yaw Error (theta_e) against steering effort, 
% this model rides the friction limits (mu) without chattering or spinning out.
% =========================================================================
clear; clc;

%% 1. HIGH-FIDELITY PLANT: VEHICLE GEOMETRY & MASS
% The LQR controller relies heavily on these physical parameters to build 
% an accurate internal prediction model of the vehicle's lateral dynamics.
L = 0.34; r = 0.054; mass = 8.0; 
a = 0.21; b = 0.13; h = 0.11;

% Yaw Inertia (Izz) and Cornering Stiffness (Cy_f, Cy_r)
Izz = (mass / 12) * (L^2 + 0.20^2); 
Cy_f = 100; Cy_r = 100; Af = 0.03; Fznom = mass * 9.81;                
mu = 0.6; g = 9.81;

%% 2. TRACK GENERATION & SPATIAL INTERPOLATION
track_file = 'Austin.csv';
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
file_path = fullfile(project_dir, 'resources', track_file);
raw_track = readmatrix(file_path);

X_scaled = raw_track(:, 1) * 0.1; Y_scaled = raw_track(:, 2) * 0.1;

dist_array = zeros(length(X_scaled), 1);
for i = 2:length(X_scaled)
    dist_array(i) = dist_array(i-1) + sqrt((X_scaled(i) - X_scaled(i-1))^2 + (Y_scaled(i) - Y_scaled(i-1))^2);
end

dist_interp = 0:0.05:dist_array(end);
X_interp = spline(dist_array, X_scaled, dist_interp)';
Y_interp = spline(dist_array, Y_scaled, dist_interp)';
path_waypoints = [X_interp, Y_interp];
num_points = length(path_waypoints);

%% 3. KINEMATIC PREVIEW & CURVATURE PROFILING (SMART THROTTLE)
% Generates the curvature (kappa) and maps it to a dynamic velocity profile.
% The "Smart Throttle" utilizes this profile to proactively brake BEFORE 
% entering a corner, managing longitudinal weight transfer and ensuring the 
% front slip angle (alpha_f) never exceeds the physical saturation limits.

raw_v_ref = zeros(num_points, 1);
curvature_array = zeros(num_points, 1); 

max_straight_speed = 20.0;
min_corner_speed = 4.0;
aggression_factor = 1.2; 

for i = 2:(num_points-1)
    x1 = path_waypoints(i-1,1); y1 = path_waypoints(i-1,2);
    x2 = path_waypoints(i,1);   y2 = path_waypoints(i,2);
    x3 = path_waypoints(i+1,1); y3 = path_waypoints(i+1,2);
    
    area = 0.5 * abs(x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2));
    a_l = sqrt((x2-x1)^2 + (y2-y1)^2); b_l = sqrt((x3-x2)^2 + (y3-y2)^2); c_l = sqrt((x3-x1)^2 + (y3-y1)^2);
    
    if area < 1e-6, kappa = 0; else, kappa = (4 * area) / (a_l * b_l * c_l); end
    
    cross_p = (x2-x1)*(y3-y2) - (y2-y1)*(x3-x2);
    curvature_array(i) = kappa * sign(cross_p);
    
    if kappa < 0.01
        raw_v_ref(i) = max_straight_speed;
    else
        radius = 1 / kappa;
        raw_v_ref(i) = max(min_corner_speed, min(max_straight_speed, sqrt(mu * g * radius)));
    end
end
raw_v_ref(1) = raw_v_ref(2); raw_v_ref(end) = raw_v_ref(end-1);
v_profile = raw_v_ref * aggression_factor;
v_profile(1) = 0.01; 

% Backward/Forward Propagation (Braking and Acceleration limits)
a_dec = 4.0; ds = 0.05;   
for i = (num_points-1):-1:1
    v_profile(i) = min(v_profile(i), sqrt(v_profile(i+1)^2 + 2 * a_dec * ds));
end
a_acc = 3.0; 
for i = 2:num_points
    v_profile(i) = min(v_profile(i), sqrt(v_profile(i-1)^2 + 2 * a_acc * ds));
end
v_ref_array = v_profile;

%% 4. LQR STATE-SPACE MATRIX FORMULATION & GAIN SCHEDULING
% State Vector: x = [e_cg, e_cg_dot, theta_e, theta_e_dot]^T
% e_cg: Cross-track error | theta_e: Heading error
% Because the Continuous-Time Algebraic Riccati Equation (CARE) is heavily 
% dependent on longitudinal velocity (v), we pre-compute the optimal gain 
% matrix (K) across a schedule of velocity nodes.

V_nodes = [1.0, 5.0, 10.0, 15.0]; 
K_array = zeros(length(V_nodes), 4);

% =========================================================================
% RESTORED TO PROVEN 0.786m CTE TUNING
% Q-Matrix penalizes state errors (High Q(1,1) strictly enforces low CTE)
% R-Matrix penalizes actuator effort (High R smooths out the steering chatter)
Q = diag([20, 1, 10, 1]); 
R = 5; 
% =========================================================================

% Feedforward Steering Gain (Kv) accounts for steady-state cornering understeer
Kv = (mass * b / (2 * L * Cy_f)) - (mass * a / (2 * L * Cy_r));

for i = 1:length(V_nodes)
    v = V_nodes(i);
    
    % A-Matrix: System Dynamics (incorporating Cornering Stiffness and Mass)
    A = [0, 1, 0, 0;
         0, -(2*Cy_f + 2*Cy_r)/(mass*v), (2*Cy_f + 2*Cy_r)/mass, (-2*Cy_f*a + 2*Cy_r*b)/(mass*v);
         0, 0, 0, 1;
         0, -(2*Cy_f*a - 2*Cy_r*b)/(Izz*v), (2*Cy_f*a - 2*Cy_r*b)/Izz, -(2*Cy_f*a^2 + 2*Cy_r*b^2)/(Izz*v)];
     
    % B-Matrix: Input Dynamics (incorporating Steering Angle constraints)
    B = [0; 2*Cy_f/mass; 0; 2*Cy_f*a/Izz];
    
    % Compute the optimal gain matrix for this specific velocity node
    K_array(i, :) = lqr(A, B, Q, R);
end

total_track_length = dist_array(end);
avg_expected_speed = mean(v_ref_array);

% Dynamic simulation timeout calculation
t_sim_stop = (total_track_length / avg_expected_speed) * 2;

%% 5. WORKSPACE INITIALIZATION LOG
fprintf('\n==================================================\n');
fprintf(' M04: LQR + FEEDFORWARD + SMART THROTTLE INITIALIZED\n');
fprintf('==================================================\n');
fprintf(' Test Configuration:\n');
fprintf('   Max Straight Speed : %.1f m/s\n', max_straight_speed);
fprintf('   Min Corner Speed   : %.1f m/s\n', min_corner_speed);
fprintf('   Aggression Factor  : %.1fx\n', aggression_factor);
fprintf(' Optimal Control Parameters:\n');
fprintf('   LQR Q-Matrix (CTE) : %.1f\n', Q(1,1));
fprintf('   LQR Q-Matrix (Yaw) : %.1f\n', Q(3,3));
fprintf('   LQR R-Matrix (Str) : %.1f\n', R);
fprintf('   Smart Throttle     : Active (Deadband & Asymmetric)\n');
fprintf('   Friction Limit     : Enabled (mu = %.1f)\n', mu);
fprintf('==================================================\n\n');