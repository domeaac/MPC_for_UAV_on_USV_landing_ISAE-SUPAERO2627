Simulink Models:

boatModel.slx contains (who would have guessed) a/maybe several USV model(s) to try robustness on

model_NLMPC_LMPC_yaw0.slx contains both a Non Linear MPC and a Linear MPC based on a quaternion 
based (inertial) drone dynamics model which is linearized in hover

model_NLMPC_LMPC_yaw0.slx contains both a Non Linear MPC and a Linear MPC based on a quaternion 
based (inertial) drone dynamics model which is linearized in hover and constantly corrected for yaw

[WIP] model_LMPC_yaw0_relativeDynamics contains a Linear MPC based on a quaternion based drone-USV
relative dynamics model, described via a USV-centered reference frame and constantly corrected
for yaw 