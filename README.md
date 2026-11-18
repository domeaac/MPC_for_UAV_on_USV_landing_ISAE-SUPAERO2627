# MPC for UAV landing on a USV

Model Predictive Control of a quadrotor landing autonomously on a moving USV (unmanned surface vehicle) whose deck is excited by waves, simulated in MATLAB/Simulink.

Research project carried out with the DCAS department at ISAE-SUPAERO, as part of my Diplôme d'ingénieur coursework.

## Overview

- **Plant:** 6-DOF quaternion rigid-body model of the drone. A second rigid body models the USV, with a wave-torque model on the boat and a spring-damper contact force for the landing deck.
- **Controller:** linear MPC built on a hover linearisation of the drone. The current model uses **relative** position and velocity (drone with respect to the USV) and **absolute** attitudes.
- **Gain scheduling:** four MPCs share the same plant and use different weights and constraints for each phase: `APPROACH → CLOSEUP → LANDING → FLARE`. A Stateflow chart switches between them based on the drone-USV distance.
- **Custom MPC (WIP):** `myLMPC.m` is a from-scratch linear MPC that will replace the MPC Toolbox blocks.

## Quick start

1. Open MATLAB (R2024b) in the repository root.
2. Run `main.m`.

`main.m` adds `src/` to the path, runs every setup script, builds the four scheduled MPC objects (`mpcAPPROACH`, `mpcCLOSEUP`, `mpcLANDING`, `mpcFLARE`) and simulates the model named in `model` for `StopTime` seconds (100 s by default). The results are returned in `out`.

To run a different model, change `model` in `main.m`. To use the single (unscheduled) linear MPC or the nonlinear MPC instead, uncomment `setupLinearController` or `setupController` in the same file.

## Simulink models

| Model | Description |
|---|---|
| `model_LMPC_yawVarReallyRelative2.slx` | **Current main model.** Scheduled linear MPC with relative drone-USV position and velocity, absolute attitudes, a USV with waves and a rigid landing platform. |
| `model_custom_LMPC.slx` | Copy of the main model used to integrate the custom MPC (`myLMPC.m`). It still uses the toolbox MPC blocks for now. |
| `models/model_LMPC_yawVarFakeRelative.slx`, `models/model_LMPC_yawVarArtificiallyRelative.slx` | Earlier relative-frame versions, superseded by `...ReallyRelative2`. |
| `models/model_NLMPC_LMPCyaw0.slx` | NLMPC and linear MPC on the inertial quaternion drone model, linearised in hover. |
| `models/model_NLMPC_LMPCyawVar.slx` | Same as above, with the linearisation continuously corrected for yaw. |
| `boatModels.slx` | USV models used for robustness tests (run from `boatMain.m`). |

## Phase scheduling

The `Drone Controller/Chart` Stateflow chart enables exactly one of the four `Enabled MPC` subsystems at a time. The switching radii are set in `src/init/setupScheduler.m`, with hysteresis to avoid chattering between phases:

| Phase | Entered when distance | Falls back when distance |
|---|---|---|
| APPROACH | > 20.1 m | — |
| CLOSEUP | < 20 m | > 20.1 m → APPROACH |
| LANDING | < 5 m | > 6.5 m → CLOSEUP |
| FLARE | < 1 m | never (irreversible) |

The chart only falls back to an earlier phase when its `returnPossible` input is set. It also has a shadow-mode state (`settings.shadowModeTimer`) and an `onlyFlare` input.

Constraints tighten as the drone gets closer to the deck (`src/lmpc/scheduledLimitsConfig.m`):

| Phase | Max tilt | Max \|v\| (x, y, z) | Max body rate |
|---|---|---|---|
| APPROACH | 20° | 5, 5, 5 m/s | 60°/s |
| CLOSEUP | 15° | 3, 3, 1.5 m/s | 45°/s |
| LANDING | 10° | 1.5, 1.5, 0.8 m/s | 30°/s |
| FLARE | 5° | 0.5, 0.5, 0.4 m/s | 30°/s |

The inputs are hard-limited in every phase: thrust deviation ±3 N, torques ±1 N·m. The output constraints are soft, with ECR values set for each group (position, velocity, tilt, rate). The weights are in `src/lmpc/scheduledWeightsConfig.m`: position and velocity are tracked in every phase, and attitude and body rates are only weighted during LANDING and FLARE.

The toolbox's built-in state estimator is bypassed (`setEstimator(..., 'custom')`, no output-disturbance integrators). The MPC blocks receive the 12 plant states directly through their `x[k|k]` inport.

## Custom linear MPC (`myLMPC.m`)

```matlab
u = myLMPC(model, state, reference, tP, costfunc)
```

This is an implementation of the augmented-state (Δu) MPC from S. H. Żak, *An Introduction to Model-based Predictive Control (MPC)*, pp. 4–8:

1. Discretises `ss(model.A, model.B, model.C, 0)` with sample time `model.Ts`.
2. Builds the augmented model `xa = [Δx; y]` and the prediction matrices `Y = W·xa + Z·ΔU` over `tP` steps.
3. Minimises `costfunc(Y, dU, Rp)` over `ΔU` with `fminunc`, then applies only the first move: `u = state.u_prev + ΔU(1:m)`.

Inputs:

- `state` has the fields `x`, `x_prev` and `u_prev`.
- `reference` is either one `p×1` output target, which is held over the horizon, or a full stacked `p·tP×1` trajectory.
- `costfunc` is a user-supplied handle, for example:

```matlab
costfunc = @(Y, dU, Rp) (Rp - Y)' * (Rp - Y) + 0.1 * (dU' * dU);
```

Status: the function works on its own but is not yet connected to `model_custom_LMPC.slx`. It is unconstrained because `fminunc` does not handle input or output limits. The control horizon is equal to the prediction horizon.

## Repository layout

```
main.m                       entry point (scheduled linear MPC)
myLMPC.m                     custom linear MPC (WIP)
boatMain.m                   boat-only test script
src/
  init/                      setupDrone, setupEnvironment, setupPlane, setupScheduler,
                             setupSettings, setupTargetTrajectory
  boat/                      setupBoat
  lmpc/                      setupLinearController, setupScheduledLinearControllers,
                             scheduledWeightsConfig, scheduledLimitsConfig
  nlmpc/                     setupController (nlmpc), quadDynamics (state function),
                             setupMPCconfigs (Ts, horizons, base weights and limits)
  subsystems/equations/      Dynamics, Kinematics (referenced subsystems)
  subsystems/utils/          DCM2quat and other attitude utilities
  libraries/utils.slx        library containing the BOAT BOUNCING FORCE block
```

`src/subsystems/` also contains spacecraft-attitude blocks (orbit, magnetorquer, reaction wheel, SRP, etc.) that the landing models do not use.

## Main parameters

| What | Where | Value |
|---|---|---|
| MPC sample time | `setupMPCconfigs.m` | `Ts = 0.025 s` (40 Hz) |
| Prediction / control horizon | `setupMPCconfigs.m` | `P = 100`, `M = 5` steps (2.5 s / 0.125 s) |
| Drone mass, inertia | `setupDrone.m` | 1 kg, diag(0.1, 0.1, 0.1) kg·m² |
| USV inertia | `setupBoat.m` | diag(25, 5, 6) kg·m² |
| Deck contact stiffness / damping | `setupPlane.m` | `K = 10`, `C = 100` |
| Target trajectory | `setupTargetTrajectory.m` | circle, `R = 5 m`, `w = 0.2 rad/s` |

## Requirements

MATLAB/Simulink R2024b with:
- Model Predictive Control Toolbox
- Optimization Toolbox (needed by `nlmpc` and by `fminunc` in `myLMPC`)
- Control System Toolbox
- Stateflow (phase scheduler)
- Aerospace Blockset (DCM ↔ quaternion blocks)
- UAV Toolbox (UAV Animation block and 3D attitude plots)

## Conventions

- Quaternions are **scalar-last**: `[qx qy qz qw]`. Aerospace Blockset blocks use scalar-first, so the models convert through the DCM before using them.
- The linear MPC state is `[x y z vx vy vz qx qy qz p q r]`. Only the vector part of the quaternion is used, which is valid near hover.
- The MPC inputs are `[ΔT τx τy τz]`, where ΔT is the thrust deviation from hover.

## Author

Domenico Raffaele Acierno, Politecnico di Milano / ISAE-SUPAERO
