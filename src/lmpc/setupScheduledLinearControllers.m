function controllers = setupScheduledLinearControllers(drone, environment)
cfg     = setupMPCconfigs();
weights = cfg.weightsScheduled;
phases  = {'APPROACH', 'CLOSEUP', 'LANDING', 'FLARE'};

plant   = buildHoverPlant(drone, environment);
baseMPC = buildBaseMPC(plant, cfg);

controllers = struct();
for k = 1:numel(phases)
    controllers.(phases{k}) = setWeights(baseMPC, weights.out(k,:), weights.rate(k,:));
end
end


function plant = buildHoverPlant(drone, environment)
m = drone.mass; g = environment.g; I = drone.inertia_diagonalForm;

% Linearizzazione all'hover, stati [x y z vx vy vz qx qy qz p q r]
A = zeros(12);
A(1:3, 4:6)   = eye(3);
A(4, 8)       =  2*g;
A(5, 7)       = -2*g;
A(7:9, 10:12) = 0.5*eye(3);

B = zeros(12, 4);
B(6, 1)       = 1/m;
B(10:12, 2:4) = diag(1./I);

plant = ss(A, B, eye(12), 0);
end


function mpcobj = buildBaseMPC(plant, cfg)
mpcobj = mpc(plant, cfg.Ts, cfg.P, cfg.M);

mpcobj.MV(1).Min = cfg.limits.thrust(1);
mpcobj.MV(1).Max = cfg.limits.thrust(2);
for i = 2:4
    mpcobj.MV(i).Min = cfg.limits.torque(1);
    mpcobj.MV(i).Max = cfg.limits.torque(2);
end

if ~isempty(cfg.limits.tilt)
    qlim = sin(cfg.limits.tilt / 2);
    for i = 7:8
        mpcobj.OV(i).Min = -qlim;
        mpcobj.OV(i).Max =  qlim;
    end
end
end


function mpcobj = setWeights(mpcobj, outWeights, rateWeights)
mpcobj.Weights.OutputVariables          = outWeights;
mpcobj.Weights.ManipulatedVariablesRate = rateWeights;
end
