function out = setupBoat()

boat.mass = 1; % kg
boat.m = boat.mass;  

boat.x0 = [0; 0; 0]; % m

boat.v0 = [0; 0; 0]; % m/s

boat.w0 = [0; 0; 0]; % rad/s

boat.inertia = [25, 0, 0; 
    0, 5, 0; 
    0, 0, 6]; % kg/m^2

boat.inertia_diagonalForm = [25; 5; 6]; % kg/m^2

boat.initialDCM = [1, 0, 0;
    0, 1, 0;
    0, 0, 1]; % []

out = boat;

end





