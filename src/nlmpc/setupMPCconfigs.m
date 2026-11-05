function cfg = setupMPCconfigs()

% TIMING
cfg.Ts = 0.025;      % sample time [s], meaning that the controller runs at 1/Ts [Hz].
cfg.P  = 100;        % prediction horizon in terms of step.
                    % NOTE: Se devo controllare in pos e vel devo tenere a mente che
                    %       vel è il risultato di 4 integrators (torque -> rate -> tilt (che crea una forza) e dunque acceleration -> vel -> pos), so the
                    %       quindi MPC has to see far enough ahead to know that tilting pays off.
cfg.M  = 5;         % control horizon [steps]: number of free input moves optimised.


%%  WEIGHTS FOR NORMAL CONTROLLERS
% Output weights: cost of tracking error on each output [x y z vx vy vz qx qy qz].
%   qx qy qz are the quaternion vector part.
%   1 = tracked normally, 0 = not tracked (free to move as the MPC needs).
cfg.weights.out  = [1 1 1  1 1 1  0 0 0  0 0 0];
                    %   x y z    : 1   tracked with full priority
                    %   vx vy vz : 1   tracked with full priority
                    %   qx qy qz : 0   not tracked

cfg.weights.rate = [0.1 0.1 0.1 0.1];

%% WEIGHTS FOR SCHEDULED CONTROLLERS
cfg.weightsScheduled = scheduledWeightsConfig();
cfg.limitsScheduled = scheduledLimitsConfig();


% LIMITS
% The oprimizer sees these as hard constraints
cfg.limits.thrust = [-3 3];         % dT [N]
cfg.limits.torque = [-5 5];   % tau_x, tau_y, tau_z [N m]

% It is anw output limit, soft by default
cfg.limits.tilt   = deg2rad(15);

cfg.limits.vel    = [];

% Input limits (thrust, torque) are always hard.
% We are setting output limits as soft to help convergence.
% We cant have existance of a solution while imposing input and output limitation always (obvious)
cfg.limits.hard   = false;
end
