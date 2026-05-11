%% ========================================================================
% MASTER ANALYTICS ENGINE & BATCH EXPORTER (High Fidelity)
% Architecture: Automated Data Extraction, Combinatorial A/B Testing & Export
% Purpose: Processes single or multiple telemetry files. Automatically generates 
% and saves publication-ready, dark-themed visualizations for solo runs and 
% pairwise A/B comparisons to mathematically prove control model superiority.
% Includes a dedicated spatial mapping engine for MPC Obstacle Avoidance.
% =========================================================================
clc; close all;

% =========================================================================
% 0. DUAL-MODE SELECTION & WORKSPACE PREP
% =========================================================================
plot_mode = questdlg('What would you like to plot?', ...
    'Analytics Engine', 'Current Workspace (Single)', 'Saved .mat Files (Compare)', 'Current Workspace (Single)');
if isempty(plot_mode), disp('Analytics canceled.'); return; end

% Attempt to grab global track data for reference
try
    ideal_path = evalin('base', 'path_waypoints');
    track_dist = evalin('base', 'dist_interp');
    v_target = evalin('base', 'v_ref_array');
    has_ideal_track = true;
    total_track_length = track_dist(end);
catch
    has_ideal_track = false;
    total_track_length = 0;
    fprintf('\n[WARNING]: Ideal track variables not found in base workspace.\n');
end

% Inline function: Variable-Step Solver Synchronization
% Pads asynchronous simulation arrays with NaNs to prevent timeline truncation
pad_or_trim = @(arr, target_len) [reshape(arr(1:min(length(arr), target_len)), [], 1); NaN(max(0, target_len - length(arr)), 1)];

% =========================================================================
% MODE A: SINGLE RUN (CURRENT WORKSPACE LIVE VIEW)
% =========================================================================
if strcmp(plot_mode, 'Current Workspace (Single)')
    fprintf('Extracting telemetry from current workspace...\n');
    
    try
        if exist('out', 'var')
            if isa(out.Actual_X, 'timeseries')
                actual_x = out.Actual_X.Data; actual_y = out.Actual_Y.Data;
                actual_yaw = out.Actual_YawRate.Data; steer_cmd = out.Steering_Angle.Data; 
                if isprop(out, 'tout'), time_sim = out.tout; else, time_sim = out.Actual_X.Time; end
            else
                actual_x = out.Actual_X; actual_y = out.Actual_Y;
                actual_yaw = out.Actual_YawRate; steer_cmd = out.Steering_Angle;
                time_sim = (0:(length(actual_x)-1))' * 0.01;
            end
            
            if isprop(out, 'CrossTrackError')
                if isa(out.CrossTrackError, 'timeseries'), cte_data = out.CrossTrackError.Data; 
                else, cte_data = out.CrossTrackError; end
            elseif has_ideal_track
                cte_data = zeros(size(actual_x));
                for k = 1:length(actual_x)
                    dists = (ideal_path(:,1) - actual_x(k)).^2 + (ideal_path(:,2) - actual_y(k)).^2;
                    cte_data(k) = sqrt(min(dists));
                end
            else
                cte_data = zeros(size(actual_x));
            end
            
            if isprop(out, 'Alpha_f'), alpha_f = out.Alpha_f.Data; else, alpha_f = []; end
            if isprop(out, 'SlipAngle'), beta = out.SlipAngle.Data; else, beta = []; end
        else
            actual_x = evalin('base', 'Actual_X.Data'); actual_y = evalin('base', 'Actual_Y.Data');
            actual_yaw = evalin('base', 'Actual_YawRate.Data'); steer_cmd = evalin('base', 'Steering_Angle.Data');
            time_sim = evalin('base', 'tout');
            
            if evalin('base', 'exist(''CrossTrackError'', ''var'')')
                cte_data = evalin('base', 'CrossTrackError.Data');
            elseif has_ideal_track
                cte_data = zeros(size(actual_x));
                for k = 1:length(actual_x)
                    dists = (ideal_path(:,1) - actual_x(k)).^2 + (ideal_path(:,2) - actual_y(k)).^2;
                    cte_data(k) = sqrt(min(dists));
                end
            else
                cte_data = zeros(size(actual_x));
            end
            
            if evalin('base', 'exist(''Alpha_f'', ''var'')'), alpha_f = evalin('base', 'Alpha_f.Data'); else, alpha_f = []; end
            if evalin('base', 'exist(''SlipAngle'', ''var'')'), beta = evalin('base', 'SlipAngle.Data'); else, beta = []; end
        end
    catch ME
        error('Signal Extraction Failed: %s', ME.message);
    end
    
    target_len = min([length(actual_x), length(actual_y), length(time_sim)]);
    actual_x = pad_or_trim(actual_x, target_len); actual_y = pad_or_trim(actual_y, target_len);
    time_sim = pad_or_trim(time_sim, target_len); cte_data = pad_or_trim(cte_data, target_len);
    actual_yaw = pad_or_trim(actual_yaw, target_len); steer_cmd = pad_or_trim(steer_cmd, target_len);
    if ~isempty(alpha_f), alpha_f = pad_or_trim(alpha_f, target_len); end
    if ~isempty(beta), beta = pad_or_trim(beta, target_len); end
    
    actual_dist = zeros(target_len, 1); actual_speed = zeros(target_len, 1);
    for i = 2:target_len
        dx = actual_x(i) - actual_x(i-1); dy = actual_y(i) - actual_y(i-1); dt = time_sim(i) - time_sim(i-1);
        step_dist = sqrt(dx^2 + dy^2); actual_dist(i) = actual_dist(i-1) + step_dist;
        if dt > 0, actual_speed(i) = step_dist / dt; else, actual_speed(i) = actual_speed(i-1); end
    end
    
    this_color = '#FA4616'; 
    fig1 = figure('Name', 'Live Trajectory', 'Color', 'k'); hold on; grid on; axis equal;
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
    if has_ideal_track, plot(ideal_path(:,1), ideal_path(:,2), 'w--', 'LineWidth', 1.5); end
    plot(actual_x, actual_y, '-', 'Color', this_color, 'LineWidth', 2);
    title('Track Telemetry & Trajectory', 'Color', 'w');
    
    fprintf('Live Workspace Plotting Complete.\n');

% =========================================================================
% MODE B: AUTOMATED MULTI-FILE OVERLAY & PAIRWISE COMPARISON
% =========================================================================
else
    disp('Select the .mat data files to compare.');
    [filenames, path] = uigetfile('*.mat', 'Select Simulation Data', 'MultiSelect', 'on');
    if isequal(filenames, 0), disp('Canceled.'); return; end
    if ischar(filenames), filenames = {filenames}; end
    num_files = length(filenames);
    
    % Theme: KMUTT Orange (#FA4616) and differentiable related shades
    colors = {'#FA4616', '#FFB300', '#FF8C00', '#E64A19', '#FFD54F'};
    
    script_dir = fileparts(mfilename('fullpath')); project_dir = fileparts(script_dir);
    data_dir = fullfile(project_dir, 'data'); if ~exist(data_dir, 'dir'), mkdir(data_dir); end
    
    % --- 1. Load All Data into Structure & Export Solo Figures ---
    models_data = struct();
    for i = 1:num_files
        temp = load(fullfile(path, filenames{i}));
        [~, name, ~] = fileparts(filenames{i});
        full_name = upper(strrep(strrep(name, 'RACE_DATA_', ''), '_', ' '));
        short_name = strrep(full_name, ' ', '');
        
        % Advanced Signal Extraction (From plot_results.m logic)
        if isfield(temp, 'out')
            d_source = temp.out;
            cx = d_source.Actual_X.Data; cy = d_source.Actual_Y.Data;
            if ismember('tout', d_source.who), ctime = d_source.tout; else, ctime = d_source.Actual_X.Time; end
            
            % Fix for 0 CTE: Attempt to pull, otherwise calculate from track
            if ismember('CrossTrackError', d_source.who)
                if isa(d_source.CrossTrackError, 'timeseries'), ccte = d_source.CrossTrackError.Data; else, ccte = d_source.CrossTrackError; end
            elseif has_ideal_track
                ccte = zeros(size(cx));
                for k = 1:length(cx)
                    dist_sq = (ideal_path(:,1) - cx(k)).^2 + (ideal_path(:,2) - cy(k)).^2;
                    ccte(k) = sqrt(min(dist_sq));
                end
            else
                ccte = zeros(size(cx)); 
            end
            
            if ismember('Actual_YawRate', d_source.who), if isa(d_source.Actual_YawRate, 'timeseries'), cr = d_source.Actual_YawRate.Data; else, cr = d_source.Actual_YawRate; end; else, cr = zeros(size(cx)); end
            if ismember('Steering_Angle', d_source.who), if isa(d_source.Steering_Angle, 'timeseries'), csteer = d_source.Steering_Angle.Data; else, csteer = d_source.Steering_Angle; end; else, csteer = zeros(size(cx)); end
            if ismember('Alpha_f', d_source.who), if isa(d_source.Alpha_f, 'timeseries'), calf = d_source.Alpha_f.Data; else, calf = d_source.Alpha_f; end; else, calf = []; end
            if ismember('SlipAngle', d_source.who), if isa(d_source.SlipAngle, 'timeseries'), cbeta = d_source.SlipAngle.Data; else, cbeta = d_source.SlipAngle; end; else, cbeta = []; end
        else
            cx = temp.actual_x; cy = temp.actual_y;
            ctime = (0:(length(cx)-1))' * 0.01;
            if isfield(temp, 'cte_data'), ccte = temp.cte_data; else, ccte = zeros(size(cx)); end
            if isfield(temp, 'actual_yaw'), cr = temp.actual_yaw; else, cr = zeros(size(cx)); end
            if isfield(temp, 'steer_cmd'), csteer = temp.steer_cmd; else, csteer = zeros(size(cx)); end
            if isfield(temp, 'alpha_f'), calf = temp.alpha_f; else, calf = []; end
            if isfield(temp, 'beta'), cbeta = temp.beta; else, cbeta = []; end
        end
        
        tlen = min([length(cx), length(cy), length(ctime)]);
        models_data(i).name = full_name;
        models_data(i).short = short_name;
        
        models_data(i).x = pad_or_trim(cx, tlen); models_data(i).y = pad_or_trim(cy, tlen);
        models_data(i).t = pad_or_trim(ctime, tlen); models_data(i).cte = pad_or_trim(ccte, tlen);
        models_data(i).steer = pad_or_trim(csteer, tlen); models_data(i).r = pad_or_trim(cr, tlen);
        models_data(i).alf = pad_or_trim(calf, tlen); models_data(i).beta = pad_or_trim(cbeta, tlen);
        
        % Distance and Speed calculation logic
        d = zeros(tlen, 1); s = zeros(tlen, 1);
        for j = 2:tlen
            seg = sqrt((models_data(i).x(j)-models_data(i).x(j-1))^2 + (models_data(i).y(j)-models_data(i).y(j-1))^2);
            dt = models_data(i).t(j) - models_data(i).t(j-1);
            d(j) = d(j-1) + seg;
            if dt > 0, s(j) = seg/dt; else, s(j) = s(j-1); end
        end
        models_data(i).dist = d; models_data(i).speed = s;
        
        % --- Export Solo Figures with Logic Check ---
        fprintf('Saving Solo Analytics for: %s\n', short_name);
        m = models_data(i); c = colors{1};
        
        si1 = figure('Visible', 'off', 'Color', 'k'); hold on; grid on; axis equal;
        set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
        title(['Trajectory: ', m.name], 'Color', 'w', 'Interpreter', 'none'); xlabel('Global X [m]'); ylabel('Global Y [m]');
        
        si2 = figure('Visible', 'off', 'Color', 'k');
        sax_v = subplot(2,1,1,'Parent', si2); hold(sax_v, 'on'); grid(sax_v, 'on');
        set(sax_v, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
        title(sax_v, 'Velocity Profile', 'Color', 'w'); ylabel(sax_v, 'Speed [m/s]');
        sax_e = subplot(2,1,2,'Parent', si2); hold(sax_e, 'on'); grid(sax_e, 'on');
        set(sax_e, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
        title(sax_e, 'Cross Track Error (CTE)', 'Color', 'w'); xlabel(sax_e, 'Distance [m]'); ylabel(sax_e, 'Error [m]');
        
        si3 = figure('Visible', 'off', 'Color', 'k'); hold on; grid on;
        set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
        title(['Tire Slip Angle \alpha_f: ', m.name], 'Color', 'w', 'Interpreter', 'tex'); xlabel('Time [s]'); ylabel('Front Slip Angle [rad]');
        
        si4 = figure('Visible', 'off', 'Color', 'k'); hold on; grid on;
        set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
        title(['Phase Portrait (\beta vs r): ', m.name], 'Color', 'w', 'Interpreter', 'tex'); xlabel('Slip Angle \beta [rad]'); ylabel('Yaw Rate r [rad/s]');
        
        si5 = figure('Visible', 'off', 'Color', 'k'); hold on; grid on;
        set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
        title(['Actuator Steering Effort: ', m.name], 'Color', 'w', 'Interpreter', 'none'); xlabel('Time [s]'); ylabel('Angle [deg]');
        
        if has_ideal_track
            plot(si1.CurrentAxes, ideal_path(:,1), ideal_path(:,2), 'w-', 'LineWidth', 1.5, 'DisplayName', 'Ideal Line');
            plot(sax_v, track_dist, v_target, 'w-', 'LineWidth', 1.5, 'DisplayName', 'Target');
        end
        
        plot(si1.CurrentAxes, m.x, m.y, 'Color', c, 'LineWidth', 2, 'DisplayName', m.name);
        plot(sax_v, m.dist, m.speed, 'Color', c, 'LineWidth', 2, 'DisplayName', m.name);
        plot(sax_e, m.dist, abs(m.cte), 'Color', c, 'LineWidth', 2, 'DisplayName', m.name);
        
        if ~isempty(m.alf), plot(si3.CurrentAxes, m.t, m.alf, 'Color', c, 'LineWidth', 1.5, 'DisplayName', m.name); end
        if ~isempty(m.beta), plot(si4.CurrentAxes, m.beta, m.r, 'Color', c, 'LineWidth', 1.5, 'DisplayName', m.name); end
        
        plot(si5.CurrentAxes, m.t, m.steer*(180/pi), 'Color', c, 'LineWidth', 1.2, 'DisplayName', m.name);
        
        exportgraphics(si1, fullfile(data_dir, [short_name, '_SOLO_01_Traj.png']), 'BackgroundColor', 'k');
        exportgraphics(si2, fullfile(data_dir, [short_name, '_SOLO_02_Perf.png']), 'BackgroundColor', 'k');
        exportgraphics(si3, fullfile(data_dir, [short_name, '_SOLO_03_Tire.png']), 'BackgroundColor', 'k');
        exportgraphics(si4, fullfile(data_dir, [short_name, '_SOLO_04_Phase.png']), 'BackgroundColor', 'k');
        exportgraphics(si5, fullfile(data_dir, [short_name, '_SOLO_05_Actuator.png']), 'BackgroundColor', 'k');
        
        close(si1, si2, si3, si4, si5);
    end
    
    % --- 2. Create Combinations (A/B Testing Matrices) ---
    % Automatically generates pair-wise comparisons to prove Model N+1 > Model N
    plot_groups = {1:num_files};
    if num_files >= 2
        pairs = nchoosek(1:num_files, 2);
        for p = 1:size(pairs, 1), plot_groups{end+1} = pairs(p, :); end
    end
    
    % --- 3. Batch Plotting Loop ---
    for g = 1:length(plot_groups)
        idx = plot_groups{g};
        if length(idx) > 2
            prefix_label = 'ALL MODELS';
            prefix_file = 'ALL_MODELS';
        else
            prefix_label = sprintf('%s VS %s', models_data(idx(1)).short, models_data(idx(2)).short);
            prefix_file = sprintf('%s_vs_%s', models_data(idx(1)).short, models_data(idx(2)).short);
        end
        
        fprintf('Processing Batch: %s\n', prefix_file);
        
        f1 = figure('Visible', 'off', 'Color', 'k'); hold on; grid on; axis equal;
        set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
        title(['Trajectory Overlay: ', prefix_label], 'Color', 'w', 'Interpreter', 'none'); xlabel('X [m]'); ylabel('Y [m]');
        
        f2 = figure('Visible', 'off', 'Color', 'k');
        ax_v = subplot(2,1,1,'Parent', f2); hold(ax_v, 'on'); grid(ax_v, 'on');
        set(ax_v, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
        title(ax_v, 'Velocity Comparison', 'Color', 'w'); ylabel(ax_v, 'Speed [m/s]');
        
        ax_e = subplot(2,1,2,'Parent', f2); hold(ax_e, 'on'); grid(ax_e, 'on');
        set(ax_e, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
        title(ax_e, 'Absolute CTE Comparison', 'Color', 'w'); xlabel(ax_e, 'Distance [m]'); ylabel(ax_e, 'Error [m]');
        
        f3 = figure('Visible', 'off', 'Color', 'k'); hold on; grid on;
        set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
        title(['Tire Dynamics \alpha_f: ', prefix_label], 'Color', 'w', 'Interpreter', 'tex'); xlabel('Time [s]'); ylabel('Front Slip Angle [rad]');
        
        f4 = figure('Visible', 'off', 'Color', 'k'); hold on; grid on;
        set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
        title(['Stability Phase Portrait Overlay (\beta vs r): ', prefix_label], 'Color', 'w', 'Interpreter', 'tex'); xlabel('Slip Angle \beta [rad]'); ylabel('Yaw Rate r [rad/s]');
        
        f5 = figure('Visible', 'off', 'Color', 'k'); hold on; grid on;
        set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
        title(['Actuator Effort Comparison: ', prefix_label], 'Color', 'w', 'Interpreter', 'none'); xlabel('Time [s]'); ylabel('Steer Angle [deg]');
        
        if has_ideal_track
            plot(f1.CurrentAxes, ideal_path(:,1), ideal_path(:,2), 'w-', 'LineWidth', 1.5, 'DisplayName', 'Ideal');
            plot(ax_v, track_dist, v_target, 'w-', 'LineWidth', 1.5, 'DisplayName', 'Target');
        end
        
        for k = 1:length(idx)
            m = models_data(idx(k)); c = colors{mod(k-1, length(colors))+1};
            
            plot(f1.CurrentAxes, m.x, m.y, 'Color', c, 'LineWidth', 2, 'DisplayName', m.name);
            plot(ax_v, m.dist, m.speed, 'Color', c, 'LineWidth', 2, 'DisplayName', m.name);
            plot(ax_e, m.dist, abs(m.cte), 'Color', c, 'LineWidth', 2, 'DisplayName', m.name);
            
            if ~isempty(m.alf), plot(f3.CurrentAxes, m.t, m.alf, 'Color', c, 'LineWidth', 1.5, 'DisplayName', m.name); end
            if ~isempty(m.beta), plot(f4.CurrentAxes, m.beta, m.r, 'Color', c, 'LineWidth', 1.5, 'DisplayName', m.name); end
            
            plot(f5.CurrentAxes, m.t, m.steer*(180/pi), 'Color', c, 'LineWidth', 1.2, 'DisplayName', m.name);
        end
        
        % Legends with White Text
        all_axes = {f1.CurrentAxes, ax_v, ax_e, f3.CurrentAxes, f4.CurrentAxes, f5.CurrentAxes};
        for f_idx = 1:length(all_axes)
            if ~isempty(get(all_axes{f_idx}, 'Children'))
                lgd = legend(all_axes{f_idx}, 'Location', 'best');
                set(lgd, 'TextColor', 'w', 'Color', 'none', 'EdgeColor', 'w');
            end
        end
        
        % Export Overlays
        exportgraphics(f1, fullfile(data_dir, [prefix_file, '_01_Traj.png']), 'BackgroundColor', 'k');
        exportgraphics(f2, fullfile(data_dir, [prefix_file, '_02_Perf.png']), 'BackgroundColor', 'k');
        exportgraphics(f3, fullfile(data_dir, [prefix_file, '_03_Tire.png']), 'BackgroundColor', 'k');
        exportgraphics(f4, fullfile(data_dir, [prefix_file, '_04_Phase.png']), 'BackgroundColor', 'k');
        exportgraphics(f5, fullfile(data_dir, [prefix_file, '_05_Actuator.png']), 'BackgroundColor', 'k');
        
        close(f1, f2, f3, f4, f5);
    end
    fprintf('\nBatch Comparison Complete.\n');
end

%% ========================================================================
% MPC OBSTACLE AVOIDANCE ANALYTICS (High-Fidelity Trajectory Map)
% Architecture: Artificial Potential Field (APF) Spatial Visualization
% Purpose: Generates the definitive visual proof that the MPC correctly 
% predicts and maneuvers around dynamic obstacles without violating the 
% 5-degree slip constraint or leaving the track boundaries.
% =========================================================================
clc; close all;

% --- 1. Data Extraction (MPC Specific) ---
try
    if exist('out', 'var')
        actual_x = out.Actual_X.Data;
        actual_y = out.Actual_Y.Data;
    else
        actual_x = evalin('base', 'Actual_X.Data');
        actual_y = evalin('base', 'Actual_Y.Data');
    end
    
    ideal_path = evalin('base', 'path_waypoints');
    has_ideal = true;
catch
    has_ideal = false;
    error('Required trajectory data (Actual_X, Actual_Y, path_waypoints) not found.');
end

% --- 2. Figure Setup (Publication Dark Theme) ---
fig_traj = figure('Name', 'MPC Obstacle Avoidance Telemetry', 'Color', 'k', 'Position', [100, 100, 900, 700]);
hold on; grid on; axis equal;
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.15);

% --- 3. Plotting Logic ---
% A. Ideal Racing Line
if has_ideal
    plot(ideal_path(:,1), ideal_path(:,2), 'w--', 'LineWidth', 1.2, 'DisplayName', 'Ideal Racing Line');
end

% B. MPC Actual Trajectory (KMUTT Orange)
this_color = '#FA4616'; 
plot(actual_x, actual_y, 'Color', this_color, 'LineWidth', 2.5, 'DisplayName', 'MPC Trajectory');

% C. Obstacle Visualization (The Stalled Vehicles & APF Boundaries)
if evalin('base', 'exist(''enable_obstacle'', ''var'')') && evalin('base', 'enable_obstacle') == 1
    num_obs = evalin('base', 'num_obstacles');
    obs_x_val = evalin('base', 'obs_x'); 
    obs_y_val = evalin('base', 'obs_y');
    
    % Plot obstacles as Red Pentagrams
    plot(obs_x_val(1:num_obs), obs_y_val(1:num_obs), 'rp', ...
        'MarkerFaceColor', '#FF0000', ...
        'MarkerEdgeColor', 'w', ...
        'MarkerSize', 15, ...
        'DisplayName', 'Stalled Vehicles (Obstacles)');
    
    % Visual safety buffer around obstacles (Represents the repulsive APF)
    theta = linspace(0, 2*pi, 50);
    buffer_radius = 2.0; % 2m radial safety limit
    for i = 1:num_obs
        bx = obs_x_val(i) + buffer_radius*cos(theta);
        by = obs_y_val(i) + buffer_radius*sin(theta);
        plot(bx, by, 'r:', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    end
end

% --- 4. Annotation & Styling ---
title('MPC Path Tracking & Obstacle Avoidance', 'Color', 'w', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Global X [m]', 'Color', 'w');
ylabel('Global Y [m]', 'Color', 'w');

lgd = legend('Location', 'best');
set(lgd, 'TextColor', 'w', 'Color', 'none', 'EdgeColor', 'w');

% --- 5. Export ---
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
data_dir = fullfile(project_dir, 'data');

if ~exist(data_dir, 'dir'), mkdir(data_dir); end
save_path = fullfile(data_dir, 'MPC_Obstacle_Trajectory.png');

exportgraphics(fig_traj, save_path, 'BackgroundColor', 'k', 'Resolution', 300);
fprintf('High-fidelity MPC trajectory image saved to: %s\n', save_path);