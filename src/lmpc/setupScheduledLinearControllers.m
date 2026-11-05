function controllers = setupScheduledLinearControllers(drone, environment)
cfg     = setupMPCconfigs();
weights = cfg.weightsScheduled;
limits  = cfg.limitsScheduled;
phases  = {'APPROACH', 'CLOSEUP', 'LANDING', 'FLARE'};

plant   = buildHoverPlant(drone, environment);
baseMPC = mpc(plant, cfg.Ts, cfg.P, cfg.M);

controllers = struct();
for k = 1:numel(phases)
    w.out  = weights.out(k,:);
    w.rate = weights.rate(k,:);

    lim.thrust = limits.thrust(k,:);
    lim.torque = limits.torque(k,:);
    lim.tilt   = limits.tilt(k);
    lim.vel    = limits.vel(k,:);
    lim.posMin = limits.posMin(k,:);
    lim.posMax = limits.posMax(k,:);
    lim.angRate = limits.angRate(k,:);
    lim.ecr    =  limits.ecr(k,:);

    obj = baseMPC;
    obj = setWeights(obj, w);
    obj = setLimits(obj, lim);

    controllers.(phases{k}) = obj;
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


function mpcobj = setWeights(mpcobj, w)
mpcobj.Weights.OutputVariables          = w.out;
mpcobj.Weights.ManipulatedVariablesRate = w.rate;
end


function mpcobj = setLimits(mpcobj, lim)
THRUST = 1;
TORQUE = 2:4;
POS    = 1:3;
VEL    = 4:6;
TILT   = 7:8;
RATE   = 10:12;

mpcobj.MV(THRUST).Min = lim.thrust(1);
mpcobj.MV(THRUST).Max = lim.thrust(2);

for i = TORQUE
    mpcobj.MV(i).Min = lim.torque(1);
    mpcobj.MV(i).Max = lim.torque(2);
end

for i = 1:3
    mpcobj.OV(POS(i)).Min  =  lim.posMin(i);
    mpcobj.OV(POS(i)).Max  =  lim.posMax(i);
    mpcobj.OV(VEL(i)).Min  = -lim.vel(i);
    mpcobj.OV(VEL(i)).Max  =  lim.vel(i);
    mpcobj.OV(RATE(i)).Min = -lim.angRate(i);
    mpcobj.OV(RATE(i)).Max =  lim.angRate(i);
end

qlim = sin(min(deg2rad(lim.tilt), pi) / 2);
for i = TILT
    mpcobj.OV(i).Min = -qlim;
    mpcobj.OV(i).Max =  qlim;
end
% ECR per group, same order as columns of limits.ecr: [pos vel tilt angRate]
groups = {POS, VEL, TILT, RATE};
for g = 1:numel(groups)
    for i = groups{g}
        mpcobj.OV(i).MinECR = lim.ecr(g);
        mpcobj.OV(i).MaxECR = lim.ecr(g);
    end
end

end
