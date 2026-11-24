function u = myLMPC(model, state, reference, tP, costfunc)
% Linear MPC from S. H. Zak, "An Introduction to Model-based Predictive
% Control (MPC)", pp. 4-8. 

% discrete model x(k+1) = Phi x(k) + Gamma u(k), eq. (3)
sys = c2d(ss(model.A, model.B, model.C, 0), model.Ts); % converto il modello da continuous a discreet 
Phi = sys.A; Gamma = sys.B; C = sys.C;
[n, m] = size(Gamma); p = size(C, 1);

% augmented model xa = [dx; y], eq. (11)
Phi_a   = [Phi, zeros(n, p); C*Phi, eye(p)];
Gamma_a = [Gamma; C*Gamma];
C_a     = [zeros(p, n), eye(p)];

% prediction Y = W xa(k) + Z dU, eq. (19)
W = zeros(p*tP, n + p);
Z = zeros(p*tP, m*tP);
for i = 1:tP
    W(p*(i-1)+1:p*i, :) = C_a * Phi_a^i;
    for j = 1:i
        Z(p*(i-1)+1:p*i, m*(j-1)+1:m*j) = C_a * Phi_a^(i-j) * Gamma_a;
    end
end

% minimise J over dU
xa = [state.x - state.x_prev; C*state.x];
rp = reference(:); % assicuro vettor colonna
if numel(rp) == p, rp = repmat(rp, tP, 1); end
J  = @(dU) costfunc(W*xa + Z*dU, dU, rp);

dU = fminunc(J, zeros(m*tP, 1), optimoptions('fminunc', 'Display', 'off'));
u  = state.u_prev + dU(1:m);    % receding horizon: apply only du(k)
end
