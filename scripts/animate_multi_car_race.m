%% ========================================================================
% MASTER CINEMATIC RACE ENGINE
% Architecture: Multi-Car Telemetry Synchronization & Rendering
% Features: Picture-in-Picture Minimap, Smart Camera (RMSE-Locked), 
% Multi-Obstacle Rendering, and Live Analog Telemetry HUD.
% Purpose: Generates the definitive visual proof for the comparative study. 
% Synchronizes asynchronous variable-step telemetry into a single 30 FPS 
% broadcast, allowing side-by-side visual evaluation of control models.
% =========================================================================
clc; close all; clear v;

fprintf('Initializing Multi-Car Race Engine...\n');

% =========================================================================
% 1. SELECT DATA FILES (Interactive File Picker)
% =========================================================================
disp('Select the .mat data files you want to race (e.g., Stanley, LQR, MPC).');
disp('You can select multiple files by holding Ctrl/Cmd.');
[filenames, path] = uigetfile('*.mat', 'Select Simulation Data', 'MultiSelect', 'on');

if isequal(filenames, 0)
    disp('Selection canceled.'); return;
end

if ischar(filenames)
    filenames = {filenames}; % Convert single selection to cell array
end

num_cars = length(filenames);
car_data = struct();

% Load each file and extract telemetry safely
for i = 1:num_cars
    filepath = fullfile(path, filenames{i});
    temp = load(filepath);
    
    if isfield(temp, 'out')
        c_x = temp.out.Actual_X.Data(:);
        c_y = temp.out.Actual_Y.Data(:);
        c_yaw = temp.out.Actual_Yaw.Data(:);
        c_steer = temp.out.Steering_Angle.Data(:);
        
        if isprop(temp.out.Actual_X, 'Time')
            c_time = temp.out.Actual_X.Time(:);
        else
            c_time = temp.out.tout(:);
        end
        
        % Extract CTE to calculate RMSE for dynamic Camera Lock
        if isprop(temp.out, 'CrossTrackError')
            if isa(temp.out.CrossTrackError, 'timeseries')
                c_cte = temp.out.CrossTrackError.Data(:);
            else
                c_cte = temp.out.CrossTrackError(:);
            end
            car_data(i).rmse = sqrt(mean(c_cte.^2));
        else
            car_data(i).rmse = inf; % Fallback if CTE is missing
        end
    else
        c_x = temp.actual_x(:);
        c_y = temp.actual_y(:);
        c_yaw = temp.actual_yaw(:);
        c_steer = temp.steer_cmd(:);
        c_time = (0:(length(c_x)-1))' * 0.01; 
        
        if isfield(temp, 'cte_data')
            car_data(i).rmse = sqrt(mean(temp.cte_data(:).^2));
        else
            car_data(i).rmse = inf;
        end
    end
    
    % --- SAFE PADDING INSTEAD OF AGGRESSIVE TRUNCATION ---
    % Anchor the timeline strictly to the physical X/Y coordinates to 
    % account for variable-step solver array length mismatches.
    target_len = min([length(c_x), length(c_y), length(c_time)]);
    
    car_data(i).x = c_x(1:target_len);
    car_data(i).y = c_y(1:target_len);
    car_data(i).time = c_time(1:target_len);
    
    % Safe padding for Yaw (pad with last known angle if sensor cut out early)
    car_data(i).yaw = zeros(target_len, 1);
    len_yaw = min(target_len, length(c_yaw));
    if len_yaw > 0
        car_data(i).yaw(1:len_yaw) = c_yaw(1:len_yaw);
        car_data(i).yaw(len_yaw+1:end) = c_yaw(len_yaw); 
    end
    
    % Safe padding for Steering Command
    car_data(i).steer = zeros(target_len, 1);
    len_steer = min(target_len, length(c_steer));
    if len_steer > 0
        car_data(i).steer(1:len_steer) = c_steer(1:len_steer);
        car_data(i).steer(len_steer+1:end) = c_steer(len_steer);
    end
    % ---------------------------------------------------------------
    
    [~, name, ~] = fileparts(filenames{i});
    name = strrep(name, 'RACE_DATA_', ''); 
    car_data(i).name = strrep(name, '_', ' ');
end

% Determine the "Best Car" to lock the main camera onto!
[~, best_car_idx] = min([car_data.rmse]);
fprintf('Camera Lock Acquired on: %s (Lowest RMSE)\n', car_data(best_car_idx).name);

try
    ideal_path = evalin('base', 'path_waypoints');
catch
    error('Please run an initialization script (e.g., car_init_M05_MPC.m) first to load the track waypoints into the base workspace.');
end

% =========================================================================
% 2. TIME SYNCHRONIZATION (Interpolation & Clamping)
% =========================================================================
% Because Simulink's variable-step solvers run asynchronously depending on 
% plant stiffness, we must interpolate all telemetry onto a unified master 
% timeline running at exactly 30 FPS for smooth video rendering.

max_time = 0;
for i = 1:num_cars
    if max(car_data(i).time) > max_time, max_time = max(car_data(i).time); end
end

fps = 30;
master_time = 0:(1/fps):max_time;
num_frames = length(master_time);

for i = 1:num_cars
    [unique_time, idx] = unique(car_data(i).time);
    
    % Clamp the time query so cars stop exactly where they finished
    q_time = min(master_time, max(unique_time));
    
    % Add lower clamp and restore 'extrap' to prevent floating-point NaNs
    q_time = max(q_time, min(unique_time));
    
    car_data(i).interp_x = interp1(unique_time, car_data(i).x(idx), q_time, 'linear', 'extrap');
    car_data(i).interp_y = interp1(unique_time, car_data(i).y(idx), q_time, 'linear', 'extrap');
    car_data(i).interp_yaw = interp1(unique_time, car_data(i).yaw(idx), q_time, 'linear', 'extrap');
    car_data(i).interp_steer = interp1(unique_time, car_data(i).steer(idx), q_time, 'linear', 'extrap');
    
    % Failsafe: Remove any native NaNs from violently crashed models
    car_data(i).interp_x(isnan(car_data(i).interp_x)) = 0;
    car_data(i).interp_y(isnan(car_data(i).interp_y)) = 0;
    car_data(i).interp_yaw(isnan(car_data(i).interp_yaw)) = 0;
    
    % --- PRE-CALCULATE LIVE SPEEDS (km/h) ---
    car_data(i).speed_kmh = zeros(num_frames, 1);
    for f_idx = 2:num_frames
        dx = car_data(i).interp_x(f_idx) - car_data(i).interp_x(f_idx-1);
        dy = car_data(i).interp_y(f_idx) - car_data(i).interp_y(f_idx-1);
        car_data(i).speed_kmh(f_idx) = sqrt(dx^2 + dy^2) * fps * 3.6;
    end
    car_data(i).speed_kmh(1) = car_data(i).speed_kmh(2); 
end

% =========================================================================
% 3. VIDEO SETUP & DUAL-ENVIRONMENT CREATION
% =========================================================================
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end

video_filepath = fullfile(data_dir, 'MBD_Comparative_Race.mp4');
v = VideoWriter(video_filepath, 'MPEG-4');
v.FrameRate = fps; v.Quality = 100; open(v);

fig = figure('Name', 'Multi-Car Race', 'Color', [0.05 0.05 0.05], 'Position', [50, 50, 1200, 800]);

% --- MAIN CAMERA AXES ---
ax_main = axes('Parent', fig, 'Color', [0.05 0.05 0.05], 'Position', [0.05 0.05 0.9 0.9]); 
hold(ax_main, 'on'); grid(ax_main, 'on'); axis(ax_main, 'equal');
ax_main.GridColor = [0.980 0.275 0.086]; ax_main.GridAlpha = 0.7; ax_main.LineWidth = 1.5;
ax_main.XColor = 'w'; ax_main.YColor = 'w';
xlabel(ax_main, 'Global X (m)'); ylabel(ax_main, 'Global Y (m)');
title(ax_main, 'Comparative Stability Analysis (Real-Time Race)', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'w');

% --- PIP MINIMAP AXES ---
ax_mini = axes('Parent', fig, 'Color', [0.08 0.08 0.08], 'Position', [0.72 0.65 0.25 0.25]);
hold(ax_mini, 'on'); axis(ax_mini, 'equal');
ax_mini.XTick = []; ax_mini.YTick = []; % Hide axis text
ax_mini.XColor = [0.980 0.275 0.086]; ax_mini.YColor = [0.980 0.275 0.086]; ax_mini.LineWidth = 2; ax_mini.Box = 'on';

% --- Terrazzo Floor (Massively Expanded) ---
% Generates a high-density speckle pattern to provide visual context for 
% vehicle speed and lateral translation during the animation.
fprintf('Generating Massive Terrazzo Floor (1M Specks)...\n');
min_x = min(ideal_path(:,1)) - 30; max_x = max(ideal_path(:,1)) + 30;
min_y = min(ideal_path(:,2)) - 30; max_y = max(ideal_path(:,2)) + 30;
num_specks = 1000000; 
rx = min_x + rand(num_specks, 1) * (max_x - min_x);
ry = min_y + rand(num_specks, 1) * (max_y - min_y);
rs = rand(num_specks, 1) * 6 + 1; 
c_opts = [1 1 1; 0.980 0.275 0.086; 0.05 0.05 0.05; 0.980 0.400 0.200; 0.7 0.7 0.7]; 
c_idx = randi(size(c_opts,1), num_specks, 1);
c = c_opts(c_idx, :);
scatter(ax_main, rx, ry, rs, c, 'filled', 'MarkerFaceAlpha', 0.8, 'HandleVisibility', 'off');

% --- Asphalt Track ---
track_width = 1.4; 
nx = zeros(size(ideal_path,1), 1); ny = zeros(size(ideal_path,1), 1);
for k = 1:size(ideal_path,1)-1
    dx = ideal_path(k+1,1) - ideal_path(k,1); dy = ideal_path(k+1,2) - ideal_path(k,2);
    L_norm = sqrt(dx^2 + dy^2); nx(k) = -dy/L_norm; ny(k) = dx/L_norm;
end
nx(end) = nx(end-1); ny(end) = ny(end-1);
track_x = [ideal_path(:,1) + nx*(track_width/2); flipud(ideal_path(:,1) - nx*(track_width/2))];
track_y = [ideal_path(:,2) + ny*(track_width/2); flipud(ideal_path(:,2) - ny*(track_width/2))];

% Draw Main Track
fill(ax_main, track_x, track_y, [0.15 0.15 0.15], 'EdgeColor', [0 0 0], 'LineWidth', 2, 'HandleVisibility', 'off');
plot(ax_main, ideal_path(:,1), ideal_path(:,2), 'w--', 'LineWidth', 2, 'HandleVisibility', 'off');

% Draw Minimap Track
plot(ax_mini, ideal_path(:,1), ideal_path(:,2), 'w-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
xlim(ax_mini, [min(ideal_path(:,1))-5, max(ideal_path(:,1))+5]);
ylim(ax_mini, [min(ideal_path(:,2))-5, max(ideal_path(:,2))+5]);

% --- DYNAMIC OBSTACLE VISUALIZATION ---
if evalin('base', 'exist(''enable_obstacle'', ''var'')') && evalin('base', 'enable_obstacle') == 1
    num_obs = evalin('base', 'num_obstacles');
    obs_x_val = evalin('base', 'obs_x'); obs_y_val = evalin('base', 'obs_y');
    plot(ax_main, obs_x_val(1:num_obs), obs_y_val(1:num_obs), 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 16, 'DisplayName', 'Stalled Vehicle(s)');
    plot(ax_mini, obs_x_val(1:num_obs), obs_y_val(1:num_obs), 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'HandleVisibility', 'off');
end

% --- ANALOG MULTI-NEEDLE SPEEDOMETER HUD ---
car_colors = {[0.980 0.275 0.086], [1 1 1], [0.980 0.500 0.300], [0.700 0.700 0.700], [0.980 0.180 0.050]}; 
hud_ymin = -0.2 - num_cars * 0.15; % Dynamically stretch box down for digital readouts
ax_hud = axes('Parent', fig, 'Position', [0.70 0.05 0.28 0.35]);
axis(ax_hud, 'equal'); axis(ax_hud, 'off'); hold(ax_hud, 'on');

fill(ax_hud, [-1.3 1.3 1.3 -1.3], [hud_ymin hud_ymin 1.25 1.25], [0.05 0.05 0.05], 'FaceAlpha', 0.8, 'EdgeColor', [0.980 0.275 0.086], 'LineWidth', 1.5);

% Draw Colored Analog Dial (Orange -> Light Orange -> White)
max_speed_kmh = 60;
theta_green = linspace(pi, pi - (30/max_speed_kmh)*pi, 50);
plot(ax_hud, cos(theta_green), sin(theta_green), 'Color', '#FA4616', 'LineWidth', 4);
theta_yellow = linspace(pi - (30/max_speed_kmh)*pi, pi - (45/max_speed_kmh)*pi, 50);
plot(ax_hud, cos(theta_yellow), sin(theta_yellow), 'Color', '#FC7A50', 'LineWidth', 4);
theta_red = linspace(pi - (45/max_speed_kmh)*pi, 0, 50);
plot(ax_hud, cos(theta_red), sin(theta_red), 'Color', '#FFFFFF', 'LineWidth', 4);

% Draw Ticks and Labels
for v_tick = 0:10:max_speed_kmh
    th = pi - (v_tick/max_speed_kmh)*pi;
    plot(ax_hud, [0.9*cos(th), cos(th)], [0.9*sin(th), sin(th)], 'w', 'LineWidth', 2);
    text(ax_hud, 0.75*cos(th), 0.75*sin(th), num2str(v_tick), 'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
end
text(ax_hud, 0, 0.35, 'km/h', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Pre-allocate Needles & Digital Texts
h_needles = gobjects(num_cars, 1);
h_val_texts = gobjects(num_cars, 1);

for i = 1:num_cars
    c_color = car_colors{mod(i-1, length(car_colors)) + 1};
    h_needles(i) = plot(ax_hud, [0, -0.85], [0, 0], '-', 'Color', c_color, 'LineWidth', 3); % Start at 0 km/h
    y_pos = -0.15 - (i-1)*0.15;
    text(ax_hud, -0.1, y_pos, car_data(i).name, 'Color', c_color, 'FontWeight', 'bold', 'FontSize', 10, 'HorizontalAlignment', 'right');
    h_val_texts(i) = text(ax_hud, 0.1, y_pos, '0.0', 'Color', c_color, 'FontWeight', 'bold', 'FontSize', 11, 'HorizontalAlignment', 'left', 'FontName', 'Courier New');
end
plot(ax_hud, 0, 0, 'ko', 'MarkerFaceColor', [0.980 0.275 0.086], 'MarkerSize', 14); % Center Pin

% =========================================================================
% 4. BUILD MULTIPLE CARS (Main Graphics & Minimap Dots)
% =========================================================================
a_dist = 0.21; b_dist = 0.13; track_width_car = 0.18; w_l = 0.08; w_w = 0.035; 
front_offset = -0.03; c_l = a_dist + b_dist + 0.1; c_w = 0.16;

h_hg = gobjects(num_cars, 1);
h_fl = gobjects(num_cars, 1);
h_fr = gobjects(num_cars, 1);
h_tail = gobjects(num_cars, 1);
h_mini_dots = gobjects(num_cars, 1);

for i = 1:num_cars
    c_color = car_colors{mod(i-1, length(car_colors)) + 1};
    
    % Main View Objects
    h_tail(i) = plot(ax_main, car_data(i).interp_x(1), car_data(i).interp_y(1), '-', 'Color', c_color, 'LineWidth', 2, 'DisplayName', car_data(i).name);
    
    % Use hgtransform matrices to handle local body yaw and wheel steering independently
    h_hg(i) = hgtransform('Parent', ax_main);
    fill(ax_main, [-b_dist-0.05, a_dist+0.05, a_dist+0.05, -b_dist-0.05], [-c_w/2, -c_w/2, c_w/2, c_w/2], c_color, 'Parent', h_hg(i), 'EdgeColor', 'k', 'HandleVisibility', 'off');
    fill(ax_main, [0, a_dist-0.02, a_dist-0.02, 0], [-c_w/2.5, -c_w/3, c_w/3, c_w/2.5], [0 0 0], 'Parent', h_hg(i), 'EdgeColor', 'k', 'HandleVisibility', 'off');
    fill(ax_main, [-w_l/2, w_l/2, w_l/2, -w_l/2] - b_dist, [-w_w/2, -w_w/2, w_w/2, w_w/2] + track_width_car/2, [0 0 0], 'Parent', h_hg(i), 'EdgeColor', 'k', 'HandleVisibility', 'off'); 
    fill(ax_main, [-w_l/2, w_l/2, w_l/2, -w_l/2] - b_dist, [-w_w/2, -w_w/2, w_w/2, w_w/2] - track_width_car/2, [0 0 0], 'Parent', h_hg(i), 'EdgeColor', 'k', 'HandleVisibility', 'off'); 
    
    h_fl(i) = hgtransform('Parent', h_hg(i)); h_fr(i) = hgtransform('Parent', h_hg(i)); 
    fill(ax_main, [-w_l/2, w_l/2, w_l/2, -w_l/2], [-w_w/2, -w_w/2, w_w/2, w_w/2], [0 0 0], 'Parent', h_fl(i), 'EdgeColor', 'k', 'HandleVisibility', 'off');
    fill(ax_main, [-w_l/2, w_l/2, w_l/2, -w_l/2], [-w_w/2, -w_w/2, w_w/2, w_w/2], [0 0 0], 'Parent', h_fr(i), 'EdgeColor', 'k', 'HandleVisibility', 'off');
    
    % Minimap Objects (Colored Tracker Dots)
    h_mini_dots(i) = plot(ax_mini, car_data(i).interp_x(1), car_data(i).interp_y(1), 'o', 'MarkerFaceColor', c_color, 'MarkerEdgeColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
end

legend(ax_main, 'Location', 'best');
view_padding = 4.0; 

% =========================================================================
% 5. MULTI-CAR ANIMATION LOOP
% =========================================================================
fprintf('Rendering Multi-Car Race... Please do not close the figure.\n');
try
    for f = 1:num_frames
        
        for i = 1:num_cars
            cx = car_data(i).interp_x(f); cy = car_data(i).interp_y(f);
            cyaw = car_data(i).interp_yaw(f); csteer = car_data(i).interp_steer(f);
            
            % Update Main Graphics using Transformation Matrices
            set(h_tail(i), 'XData', car_data(i).interp_x(1:f), 'YData', car_data(i).interp_y(1:f));
            set(h_hg(i), 'Matrix', makehgtform('translate', [cx, cy, 0], 'zrotate', cyaw));
            set(h_fl(i), 'Matrix', makehgtform('translate', [a_dist + front_offset, track_width_car/2, 0], 'zrotate', csteer));
            set(h_fr(i), 'Matrix', makehgtform('translate', [a_dist + front_offset, -track_width_car/2, 0], 'zrotate', csteer));
            
            % Update Minimap Dots
            set(h_mini_dots(i), 'XData', cx, 'YData', cy);
            
            % --- UPDATE LIVE ANALOG SPEEDOMETER HUD ---
            if master_time(f) < max(car_data(i).time)
                spd = car_data(i).speed_kmh(f);
                set(h_val_texts(i), 'String', sprintf('%5.1f', spd));
                th = pi - (min(spd, max_speed_kmh)/max_speed_kmh)*pi; % Map speed to angle
                set(h_needles(i), 'XData', [0, 0.85*cos(th)], 'YData', [0, 0.85*sin(th)]);
            else
                set(h_val_texts(i), 'String', 'STOPPED');
                set(h_needles(i), 'XData', [0, -0.85], 'YData', [0, 0]); % Drop needle to 0
            end
        end
        
        % --- TV BROADCAST DIRECTOR (Dynamic Camera Lock) ---
        % Find which cars are still actively driving in this frame
        active_cars = [];
        for i_cam = 1:num_cars
            if master_time(f) < max(car_data(i_cam).time)
                active_cars(end+1) = i_cam;
            end
        end
        
        if ~isempty(active_cars)
            % Dynamic Switch: Lock camera onto the most stable car (Lowest RMSE)
            active_rmses = [car_data(active_cars).rmse];
            [~, best_active_idx] = min(active_rmses);
            cam_idx = active_cars(best_active_idx);
        else
            % If everyone finished/crashed, park camera on the overall winner
            cam_idx = best_car_idx;
        end
        
        cam_x = car_data(cam_idx).interp_x(f); 
        cam_y = car_data(cam_idx).interp_y(f);
        % ---------------------------------------------------
        
        xlim(ax_main, [cam_x - view_padding, cam_x + view_padding]);
        ylim(ax_main, [cam_y - view_padding, cam_y + view_padding]);
        
        drawnow;
        writeVideo(v, getframe(fig));
    end
    
    close(v);
    fprintf('\nSuccess! MBD Cinematic Render saved to:\n%s\n', video_filepath);
    
catch ME
    close(v); rethrow(ME);
end