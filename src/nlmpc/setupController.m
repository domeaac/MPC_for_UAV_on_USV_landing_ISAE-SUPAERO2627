function controller = setupController()
cfg = setupMPCconfigs();

nlobj = nlmpc(13, 12, 4);         % stati, uscite, ingressi
nlobj.Ts                = cfg.Ts; % sample time
nlobj.PredictionHorizon = cfg.P;  % steps in pred orizion
nlobj.ControlHorizon    = cfg.M;  % control orizon

nlobj.Model.StateFcn           = "quadDynamics";
nlobj.Model.IsContinuousTime   = true;         
nlobj.Model.NumberOfParameters = 1; % un singolo parametro che in realtà è un vettore di parametri
nlobj.Model.OutputFcn          = @(x, u, params) x([1:9 11:13]);   % [x y z vx vy vz qx qy qz]

% Passo i pesi all' MPC
nlobj.Weights.OutputVariables          = cfg.weights.out;
nlobj.Weights.ManipulatedVariablesRate = cfg.weights.rate;

% Impongo limiti in attuazione
nlobj.MV(1).Min = cfg.limits.thrust(1);
nlobj.MV(1).Max = cfg.limits.thrust(2);
for i = 2:4
    nlobj.MV(i).Min = cfg.limits.torque(1);
    nlobj.MV(i).Max = cfg.limits.torque(2);
end

% Impongo il tilt max in quaternione se definito in config
if ~isempty(cfg.limits.tilt)
    qlim = sin(cfg.limits.tilt / 2);
    for i = 7:8
        nlobj.OV(i).Min = -qlim;
        nlobj.OV(i).Max =  qlim;
    end
end

controller = nlobj;
end
