%% ========================================================================
% MASTER INITIALIZATION SCRIPT: M05 Unified Model Predictive Control (MPC)
% Architecture: Explicit MPC + APF Spatial Awareness + Smart Throttle
% Purpose: The ultimate evolution of the Active Stability study. Unlike LQR, 
% MPC explicitly handles physical constraints (steering limits, tire slip limits) 
% over a prediction horizon. This phase also introduces Artificial Potential 
% Fields (APF) to prove high-speed stability during dynamic obstacle avoidance.
% =========================================================================
clear; clc;

%% 1. HIGH-FIDELITY PLANT: VEHICLE GEOMETRY & MASS
% The MPC controller utilizes these physical parameters to construct an 
% internal Linear Parameter-Varying (LPV) model, calculating the future 
% trajectory and ensuring slip states never exceed saturation limits.
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
% The Smart Throttle ensures the vehicle enters the MPC prediction horizon 
% at a survivable speed governed by the Kamm Friction Circle: v = sqrt(mu * g * R).

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

total_track_length = dist_array(end);
avg_expected_speed = mean(v_ref_array);

% Dynamic simulation timeout calculation
t_sim_stop = (total_track_length / avg_expected_speed) * 2;

%% 4. SPATIAL OBSTACLE INJECTION (Artificial Potential Fields)
% Introduces dynamic spatial awareness. 

enable_obstacle = 1; % Toggle: 1 for Obstacles, 0 for Clear Track
num_obstacles = 3;   % Specify how many obstacles to avoid (Max 10)

% Simulink Coder Failsafe: 
% The Simulink C-compiler prohibits variable-size arrays. We pad the array 
% to a fixed size of 10 to ensure seamless hardware deployment.
max_obs_limit = 10; 
obs_x = zeros(max_obs_limit, 1);
obs_y = zeros(max_obs_limit, 1);

if enable_obstacle
    % Lock obstacles to the ideal racing line between 20% and 85% of track
    min_idx = round(num_points * 0.20);
    max_idx = round(num_points * 0.85);
    
    % RNG SEED LOCK: rng(42) guarantees that stalled vehicles spawn in the EXACT 
    % same coordinates across multiple simulations. This ensures that comparative 
    % A/B testing between models remains mathematically and statistically valid.
    rng(42); 
    obs_indices = randperm(max_idx - min_idx, num_obstacles) + min_idx;
    
    obs_x(1:num_obstacles) = path_waypoints(obs_indices, 1);
    obs_y(1:num_obstacles) = path_waypoints(obs_indices, 2);
end

%% 5. WORKSPACE INITIALIZATION LOG
fprintf('\n==================================================\n');
fprintf(' M05: UNIFIED EXPLICIT MPC INITIALIZED\n');
fprintf('==================================================\n');
fprintf(' Test Configuration:\n');
fprintf('   Max Straight Speed : %.1f m/s\n', max_straight_speed);
fprintf('   Min Corner Speed   : %.1f m/s\n', min_corner_speed);
fprintf('   Aggression Factor  : %.1fx\n', aggression_factor);
fprintf(' Optimal Control Parameters:\n');
fprintf('   LPV Horizon        : Enabled (Lookahead 0.65s)\n');
fprintf('   Latency Prediction : Active (30ms Cancellation)\n');
fprintf('   Virtual ESC Limit  : 5 Degrees Slip Constraint\n');
fprintf('   Friction Limit     : Enabled (mu = %.1f)\n', mu);
fprintf(' Spatial Awareness:\n');
if enable_obstacle
    fprintf('   APF Obstacle Zone  : %d Random Stalled Vehicles Active\n', num_obstacles);
else
    fprintf('   APF Obstacle Zone  : DISABLED\n');
end
fprintf('==================================================\n\n');