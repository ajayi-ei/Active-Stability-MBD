Active Stability System for High-Speed Autonomous Driving 🏎️

📌 Executive Summary

Modern autonomous emergency maneuvers require control architectures that can navigate the narrow boundary between maximum velocity and dynamic instability. Conventional geometric path-tracking algorithms (Pure Pursuit, Stanley) rely on kinematic assumptions that neglect vehicle mass, yaw inertia, and tire-road friction limits, leading to catastrophic "spin-out" failures at high lateral accelerations.

This project utilizes a 100% Model-Based Design (MBD) paradigm to develop a proactive Active Stability System. Utilizing a high-fidelity, non-linear 3-Degree-of-Freedom (3DOF) Single Track dynamic plant, this research evaluates an evolutionary progression of five control models, culminating in an Explicit Model Predictive Controller (MPC) paired with a Smart Throttle and Artificial Potential Fields (APF) for high-speed obstacle avoidance.

📊 Benchmark Results: The MBD Proof

The following data was generated via automated batch simulation across a spline-interpolated raceline.
% Path Adherence is defined by how much of the racing time a car stays within 0.7m of the ideal line.

| MODEL        | PATH ADHERENCE | AVG SPEED | RMSE CTE |
|--------------|----------------|-----------|----------|
| MPC          | 88.8 %         | 6.10 m/s  | 0.488 m  |
| LQR          | 80.0 %         | 6.13 m/s  | 0.525 m  |
| ACTIVEYAW    | 34.3 %         | 8.47 m/s  | 3.352 m  |
| STANLEY      | 34.0 %         | 8.82 m/s  | 3.789 m  |
| PURE PURSUIT | 22.2 %         | 8.10 m/s  | 6.615 m  |

Conclusion: The optimal state-space controllers (LQR/MPC) successfully navigated the track by proactively managing the physical tire slip angle saturation limits, whereas the kinematic baseline models (Pure Pursuit/Stanley) suffered mathematical failure (spin-outs) at high velocities.

📈 Visual Analytics Suite

This repository contains a comprehensive suite of automated MBD analytics located in the /data directory, proving the mathematical superiority of the advanced controllers:

* Solo Analytics: Trajectory, Velocity Profiling, Tire Slip ($\alpha_f$), Phase Portraits ($\beta$ vs $r$), and Actuator Chatter for all 5 models.
* Global Overlays: Combined comparative plots mapping all 5 models simultaneously.
* Exhaustive Pairwise A/B Comparisons: 10 pairwise overlay sets (e.g., LQR_vs_MPC, ACTIVEYAW_vs_STANLEY) detailing the exact performance delta between any two architectures.
* MPC Specialized Analytics: High-fidelity spatial mapping (MPC_Obstacle_Trajectory.png) verifying Artificial Potential Field (APF) safety buffers during stalled-vehicle avoidance.
* Rendered Telemetry Videos: Including All Models Race, Stanley Run, MPC Run, and Stanley vs. MPC comparative broadcast.

📂 Repository Architecture

Active_Stability_MBD/
├── data/                   # Simulation telemetry (.mat), 60+ Analytics Plots (.png), and Rendered Videos (.mp4)
├── models/                 # Simulink 3DOF Non-Linear Physics Plants (.slx)
│   ├── M01_Baseline_PurePursuit.slx
│   ├── M02_Baseline_Stanley.slx
│   ├── M03_Cascaded_ActiveYaw.slx
│   ├── M04_LQR_SmartThrottle.slx
│   └── M05_MPC.slx
├── resources/              # Track matrices (Austin.csv) and Drone Tracker HUD assets (rc_car.png)
└── scripts/                # MBD Initialization, Execution, and Analytics code
    ├── car_init_M01... to M05...m # Plant and Controller Initialization Scripts
    ├── run_master_simulation.m    # Automated Batch Execution Pipeline
    ├── generate_analytics_suite.m # Solo/Pairwise A/B Plotting Engine
    ├── plot_results.m             # Live Workspace Telemetry Engine
    ├── animate_multi_car_race.m   # Synchronized 30FPS Cinematic Broadcast Engine
    └── animate_drone_view.m       # High-Fidelity Single-Car Optical Flow Tracker

🚀 How to Run the Simulation

1. Clone this repository and open the root folder in MATLAB.
2. Add the scripts/, models/, and resources/ folders to your MATLAB path.
3. Open scripts/run_master_simulation.m and run it to automatically generate telemetry.
4. Run scripts/generate_analytics_suite.m to automatically generate high-contrast solo and pairwise A/B comparative plots.
5. Run scripts/animate_multi_car_race.m or animate_drone_view.m to render the dynamic broadcast videos.

👨‍💻 Authors

* Ethan Gabriel Paredes - Lead Systems Engineer
* Eberechukwu Isaiah Ajayi - Lead Systems Engineer
* Advisor: Assoc. Prof. Dr. Benjamas Panomruttanarug
