# OpenAUV Simulation Environment

This repository contains the complete MATLAB/Simulink simulation environment for the **OpenAUV**, an omnidirectional Autonomous Underwater Vehicle. The project includes trajectory tracking control architectures and advanced disturbance estimation algorithms.

---

## 📂 File Structure

* **`inizialization.m`**: The main MATLAB script used to load vehicle dynamics coefficients, thruster allocation matrices, and control gains required for the Simulink models.
* **`OpenAUV_v1dist.slx`**: Simulink model featuring a standard PD/PID trajectory tracking control loop, simulated under environmental disturbances without the Momentum-based Estimator.
* **`OpenAUV_v2OBS.slx`**: Advanced Simulink model featuring a PID controller integrated with a **Momentum-based Disturbance Estimator** to actively reconstruct and compensate for external forces and torques.
* **`plots.m`**: Post-processing MATLAB script used to extract simulation logs and generate plots for performance evaluation.

---