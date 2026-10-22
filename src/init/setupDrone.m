function out = setupDrone()

drone.mass = 1; % kg
drone.m = drone.mass;  

drone.x0 = [0; 0; 0]; % m

drone.v0 = [0; 0; 0]; % m/s

drone.w0 = [0; 0; 0]; % rad/s

drone.inertia = [0.1, 0, 0; 
    0, 0.1, 0; 
    0, 0, 0.1]; % kg/m^2

drone.inertia_diagonalForm = [0.1; 0.1; 0.1]; % kg/m^2

drone.initialDCM = [1, 0, 0;
    0, 1, 0;
    0, 0, 1]; % []

out = drone;

end





