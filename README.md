# MPC for UAV landing on a USV

Research project carried out with the DCAS department at ISAE-SUPAERO, as part of my Diplôme d'ingénieur coursework.

## Requirements

MATLAB/Simulink with:
- Model Predictive Control Toolbox
- Optimization Toolbox (needed by `nlmpc`)
- Control System Toolbox
- UAV Toolbox (used for 3D attitude plots)

## Conventions

Quaternions are **scalar-last** `[qx qy qz qw]`. Aerospace Blockset blocks use scalar-first, so convert before using them.

## Author

Domenico Raffaele Acierno, Politecnico di Milano / ISAE-SUPAERO
