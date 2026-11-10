# FEA-Based PMSM Digital Twin — MATLAB/Simulink

Standalone MATLAB/Simulink project for the simulation and control of Permanent Magnet Synchronous Machines (PMSMs) using finite-element-analysis (FEA) lookup tables. The model includes a PWM inverter, electromagnetic characteristics, machine losses and cascaded current, speed and position control.

Developed at the National Technical University of Athens (NTUA).

---
## 1. Requirements
- MATLAB and Simulink (compatible release to be confirmed)
- Required MATLAB toolboxes and functions included in the Auxiliary folder
- Windows recommended; MATLAB/Simulink installation required


---
## 2. Quick Start
1) Keep Run_Twin.m, Twin_Builder.slx, FEA_Results.mat and Auxiliary/ in the project folder.
2) Open MATLAB and set the project folder as the current directory.
3) Run Run_Twin.m to load the FEA data and execute the Simulink model.


---
## 3. Features & Scope
- FEA-derived torque, flux-linkage, inductance and back-EMF maps
- PWM inverter and dq motor-control simulation
- Position, speed, torque and current control modes
- Machine and inverter loss estimation
- Results depend on the supplied FEA dataset and model assumptions; this is not a hardware-validated standalone executable.

---
## 4. Author & Contact
Antonios V. Sideris — PhD Student, Electrical Engineering, NTUA  
Supervisor: Prof. Antonios Kladas, Athanasios Sarigiannidis
📧 antsideris@mail.ntua.gr | anthonysideris20@gmail.com  
📧 Prof. Antonios G. Kladas: kladasel@central.ntua.gr | kladasel@mail.ntua.gr 
📧 Assistant Prof. Athanasios G. Sarigiannidis: athanasios_sarigiannidis@mail.ntua.gr
📱 +30 694 547 4883 (Author: Antonios V. Sideris)


---
## 5. License & Citation
Copyright (c) 2026 Antonios V. Sideris.
License: To be selected before public open-source release.
If you use this project in academic work, please acknowledge the author and NTUA.
