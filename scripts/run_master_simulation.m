function run_master_simulation()
%% ========================================================================
% MASTER BATCH SIMULATION MANAGER
% Architecture: Automated MBD Execution & Telemetry Pipeline
% Purpose: Programmatically initializes, executes, and extracts telemetry 
% data across the 5-model progression (Kinematic Baselines -> Optimal MPC).
% Ensures mathematically rigorous, repeatable A/B testing by eliminating 
% manual execution variables. Prepares the workspace for comparative analytics.
% =========================================================================
clc; close all;

fprintf('==================================================\n');
fprintf(' ACTIVE STABILITY: MASTER SIMULATION BATCH MANAGER\n');
fprintf('==================================================\n');
disp('Available Models: pure_pursuit, stanley, activeyaw, lqr, mpc');
disp('Example input: stanley mpc lqr');
fprintf('--------------------------------------------------\n');

% 1. SELECTIVE REGRESSION TESTING
% Allows the user to run specific models for targeted A/B comparative analysis
user_input = input('Enter the models you want to run (separated by spaces): ', 's');
if isempty(user_input)
    disp('No models selected. Exiting.');
    return;
end

% 2. TELEMETRY DATA ISOLATION
% Dynamically locates the project root and isolates generated .mat files 
% into a dedicated /data folder to maintain a clean repository structure.
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end

% Parse the input into a cell array of strings
models_to_run = split(lower(strtrim(user_input)));

% 3. AUTOMATED EXECUTION PIPELINE
for i = 1:length(models_to_run)
    current_model = models_to_run{i};
    fprintf('\n>>> PREPARING MODEL: [%s] <<<\n', upper(current_model));
    
    init_script = '';
    sim_file = '';
    save_name = sprintf('RACE_DATA_%s.mat', upper(current_model));
    
    % --- MAPPING DICTIONARY (VERIFIED FILENAMES) ---
    switch current_model
        case 'pure_pursuit'
            init_script = 'car_init_M01_PurePursuit.m';
            sim_file = 'M01_Baseline_PurePursuit'; 
            
        case 'stanley'
            init_script = 'car_init_M02_Stanley.m';
            sim_file = 'M02_Baseline_Stanley'; 
            
        case 'activeyaw'
            init_script = 'car_init_M03_ActiveYaw.m';
            sim_file = 'M03_Cascaded_ActiveYaw';
            
        case 'lqr'
            init_script = 'car_init_M04_LQR.m'; 
            sim_file = 'M04_LQR_SmartThrottle'; 
            
        case 'mpc'
            init_script = 'car_init_M05_MPC.m';
            sim_file = 'M05_MPC'; 
            
        otherwise
            fprintf('[ERROR]: Model "%s" not recognized. Skipping.\n', current_model);
            continue;
    end
    
    % --- EXECUTION SEQUENCE ---
    try
        % A. Base Workspace Initialization
        % Inject parameters (mass, Izz, track waypoints) directly into the base 
        % workspace so the Simulink high-fidelity 3DOF plant can access them.
        fprintf('1. Running Initialization: %s\n', init_script);
        evalin('base', sprintf('run(''%s'')', init_script));
        
        % B. Programmatic Physics Engine Execution
        % The sim() command runs the non-linear plant silently in the background, 
        % bypassing the GUI for maximum computational efficiency.
        fprintf('2. Executing Simulink Physics Engine: %s.slx...\n', sim_file);
        evalin('base', sprintf('out = sim(''%s'');', sim_file));
        
        % C. Automated Telemetry Extraction
        % Extracts the 'out' object (containing Actual_Yaw, Global_X, slip angles) 
        % for subsequent RMSE and Phase Portrait generation.
        fprintf('3. Saving Telemetry Data...\n');
        save_path = fullfile(data_dir, save_name);
        
        evalin('base', sprintf('save(''%s'', ''out'')', save_path));
        
        fprintf('SUCCESS: %s saved successfully!\n', save_name);
        
    catch ME
        fprintf('[CRITICAL ERROR] Failed to run %s.\n', current_model);
        fprintf('Error Details: %s\n', ME.message);
    end
end

fprintf('\n==================================================\n');
fprintf(' BATCH SIMULATION COMPLETE.\n');
fprintf(' You may now run the Analytics or Animation scripts.\n');
fprintf('==================================================\n');
end % End of function