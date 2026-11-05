clear; close all; clc;

% INIT
addpath(genpath(pwd))
%model = "model_NLMPC_LMPCyaw0";
%model = "model_NLMPC_LMPCyawVar";
model = "model_LMPC_yawVar_Relative";

StopTime = 100; % s

% SETUP SCRIPTS
settings = setupSettings();
traj = setupTargetTrajectory();
drone = setupDrone();
boat = setupBoat();
plane = setupPlane();
environment = setupEnvironment();
scheduler = setupScheduler();

%controller = setupController(); % NLMPC
%controllerLin = setupLinearController(drone, environment); % LINEAR MPC
scheduledControllersLin = setupScheduledLinearControllers(drone,environment); % SCHEDULED LINEAR MPCS
mpcAPPROACH = scheduledControllersLin.APPROACH;
mpcCLOSEUP  = scheduledControllersLin.CLOSEUP;
mpcLANDING  = scheduledControllersLin.LANDING;
mpcFLARE    = scheduledControllersLin.FLARE;

% RUN
load_system(model);
params = [drone.mass; environment.g; drone.inertia_diagonalForm];
%createParameterBus(controller, strcat(model, '/Drone Controller/Nonlinear MPC Controller'), 'paramsBus', {params});
set_param(model, 'StopTime', num2str(StopTime));
out = sim(model);
%close_system(model, 0);

% % PLOT
% pd = squeeze(out.dronePosition.Data);   if size(pd,1) ~= 3, pd = pd'; end
% pt = squeeze(out.targetPosition.Data);  if size(pt,1) ~= 3, pt = pt'; end
% q  = squeeze(out.droneQuat.Data);       if size(q,1)  ~= 4, q  = q';  end
% 
% % Traiettorie
% figure; hold on; grid on; axis equal; view(3)
% plot3(pt(1,:), pt(2,:), pt(3,:), 'k--', 'LineWidth', 1.2);
% plot3(pd(1,:), pd(2,:), pd(3,:), 'b',   'LineWidth', 1.5);
% legend('target', 'drone'); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
% title('Traiettorie')
% 
% % Ricostruzione assetto
% figure; hold on; grid on; axis equal; view(3)
% plot3(pt(1,:), pt(2,:), pt(3,:), 'k--', 'LineWidth', 1.2);
% plot3(pd(1,:), pd(2,:), pd(3,:), 'b',   'LineWidth', 1.0);
% k = 1:20:size(pd,2);
% plotTransforms(pd(:,k)', q(:,k)', 'MeshFilePath', 'multirotor.stl', 'FrameSize', 0.5);
% xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
% title('Assetto lungo la traiettoria')
