function out = setupDrone()

drone.mass = 1; % kg
drone.m = drone.mass;  

drone.x0 = [0; 0; 0]; % m

drone.v0 = [0; 0; 0]; % m/s

drone.w0 = [0; 0; 0]; % rad/s

% With I = 1 and torque +-1 N m the angular acceleration (1 rad/s^2) is too slow
% for the 0.5 s MPC horizon: the drone cannot un-tilt in time and flips.
drone.inertia = [0.1, 0, 0;
    0, 0.1, 0;
    0, 0, 0.1]; % kg/m^2

drone.inertia_diagonalForm = [0.1; 0.1; 0.1]; % kg/m^2

drone.initialDCM = [1, 0, 0;
    0, 1, 0;
    0, 0, 1]; % []

out = drone;

end





