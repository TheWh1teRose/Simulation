# NAO Sim2Real Gap

This project investigates the reality gap between simulated and real NAO robots by comparing movement trajectories and evaluating the sim-to-real transfer capabilities.

## 🎯 Project Overview

The goal is to evaluate how accurately physics simulations can replicate real-world robot behaviors. This research focuses on:
- Implementing a NAO robot model in simulation
- Comparing movement trajectories between simulated and real robots
- Quantifying the reality gap
- Documenting methodologies for effective sim-to-real transfer

## 🛠️ Methodology

### Simulation Environment
We initially planned to use MuJoCo but switched to PyBullet due to better NAO model compatibility. The simulation setup includes:
- PyBullet physics engine
- NAO robot model with fixed torso
- Position control for stable joints
- Velocity control for movement execution

### Real Robot Setup
The real-world measurements are captured using:
- AprilTag markers on the NAO robot
- Camera tracking system
- Joint trajectory recording

### Analysis Approach
The comparison between simulation and reality will be conducted by:
- Recording single joint movements
- Measuring trajectories in both environments
- Computing euclidean distances from target positions
- Analyzing movement accuracy and patterns

## 📊 Current Project State

### Completed
- Initial research and literature review
- Decision on simulation environment (PyBullet)
- Basic simulation setup
- AprilTag implementation on real NAO

### In Progress
- Trajectory recording implementation
- Movement comparison methodology
- Data collection scripts

### Planned
- Comprehensive trajectory analysis
- Reality gap quantification
- Documentation of findings

## 📚 Key References
1. Haarnoja, T. et al. (2024) ‘Learning agile soccer skills for a bipedal robot with deep reinforcement learning’, Science Robotics [Preprint]. Available at: https://doi.org/10.1126/scirobotics.adi8022.
2. Collins, J., Howard, D. and Leitner, J. (2019) ‘Quantifying the Reality Gap in Robotic Manipulation Tasks’, in 2019 International Conference on Robotics and Automation (ICRA). 2019 International Conference on Robotics and Automation (ICRA), pp. 6706–6712. Available at: https://doi.org/10.1109/ICRA.2019.8793591.
3. Muratore, F. et al. (2022) ‘Robot Learning From Randomized Simulations: A Review’, Frontiers in Robotics and AI, 9. Available at: https://doi.org/10.3389/frobt.2022.799893.
