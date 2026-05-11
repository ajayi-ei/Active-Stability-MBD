%% ========================================================================
% MASTER INITIALIZATION SCRIPT: M03 Cascaded Active Yaw Control
% Architecture: Outer Loop (Stanley Kinematics) + Inner Loop (Yaw Dynamics)
% Purpose: Introduces the first active dynamic stability interceptor. 
% By calculating the yaw rate error (psi_dot_error), this P-controller 
% injects corrective counter-steering to prevent the high-speed spin-outs 
% established in the M01 and M02 baselines.
% =========================================================================
clear; clc;

%% 1. HIGH-FIDELITY PLANT: VEHICLE GEOMETRY & MASS
% Defining the 3DOF Single Track (Bicycle) dynamic parameters.
% Note: Unlike previous models, M03 actively monitors yaw inertia (I_zz) 
% and tire friction limits (mu) to calculate the ideal bounded yaw rate.

L = 0.34; r = 0.054; mass = 8.0; 
a = 0.21; b = 0.13; h = 0.11;
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

%% 3. MASTER TEST CONTROLS
num_points = length(path_waypoints);
max_straight_speed = 20.0; 
min_corner_speed = 4.0;
aggression_factor = 1.2; 
k = 2.5; % Outer Loop Stanley Gain (Locked in from M02 optimization)

%% 4. ADVANCED VELOCITY PROFILING (Kinematic Propagation)
% Enforces the Kamm Friction Circle boundary: v_max = sqrt(mu * g * R)
raw_v_ref = zeros(num_points, 1);
for i = 2:(num_points-1)
    x1 = path_waypoints(i-1,1); y1 = path_waypoints(i-1,2);
    x2 = path_waypoints(i,1);   y2 = path_waypoints(i,2);
    x3 = path_waypoints(i+1,1); y3 = path_waypoints(i+1,2);
    area = 0.5 * abs(x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2));
    a_l = sqrt((x2-x1)^2 + (y2-y1)^2); b_l = sqrt((x3-x2)^2 + (y3-y2)^2); c_l = sqrt((x3-x1)^2 + (y3-y1)^2);
    
    if area < 1e-6, curvature = 0; else, curvature = (4 * area) / (a_l * b_l * c_l); end
    
    if curvature < 0.01
        raw_v_ref(i) = max_straight_speed;
    else
        radius = 1 / curvature;
        raw_v_ref(i) = max(min_corner_speed, min(max_straight_speed, sqrt(mu * g * radius)));
    end
end
raw_v_ref(1) = raw_v_ref(2); raw_v_ref(end) = raw_v_ref(end-1);
v_profile = raw_v_ref * aggression_factor;
v_profile(1) = 0.01; % Start-line solver fix (Prevents Div/0)

% Backward/Forward Propagation (Braking and Acceleration limits)
a_dec = 4.0; ds = 0.05;   
for i = (num_points-1):-1:1
    max_entry_speed = sqrt(v_profile(i+1)^2 + 2 * a_dec * ds);
    v_profile(i) = min(v_profile(i), max_entry_speed);
end
a_acc = 3.0; 
for i = 2:num_points
    max_exit_speed = sqrt(v_profile(i-1)^2 + 2 * a_acc * ds);
    v_profile(i) = min(v_profile(i), max_exit_speed);
end
v_ref_array = v_profile; 

%% 5. INNER LOOP: GAIN SCHEDULED ACTIVE YAW CONTROL
% Vehicle lateral dynamics are highly non-linear with respect to longitudinal 
% velocity (v_x). A fixed proportional gain will cause high-speed oscillations. 
% We utilize a 1-D Lookup Table to dynamically relax the intervention as speed increases.

Vx_breakpoints = [0.0, 5.0, 10.0, 15.0];
P_gains        = [0.80, 0.60, 0.20, 0.10]; % Locked in: Optimal balance of bounded slip and CTE

total_track_length = dist_array(end);
avg_expected_speed = mean(v_ref_array);

% Dynamic simulation timeout calculation
t_sim_stop = (total_track_length / avg_expected_speed) * 2;

%% 6. WORKSPACE INITIALIZATION LOG
fprintf('\n==================================================\n');
fprintf(' M03: CASCADED ACTIVE STABILITY INITIALIZED\n');
fprintf('==================================================\n');
fprintf(' Test Configuration:\n');
fprintf('   Max Straight Speed : %.1f m/s\n', max_straight_speed);
fprintf('   Min Corner Speed   : %.1f m/s\n', min_corner_speed);
fprintf('   Aggression Factor  : %.1fx\n', aggression_factor);
fprintf(' Active Stability Parameters:\n');
fprintf('   Outer Loop (Stanley): k = %.2f\n', k);
fprintf('   Inner Loop (P-Gain) : Scheduled 1-D Lookup\n');
fprintf('   Friction Clamp      : Enabled (mu = %.1f)\n', mu);
fprintf('==================================================\n\n');