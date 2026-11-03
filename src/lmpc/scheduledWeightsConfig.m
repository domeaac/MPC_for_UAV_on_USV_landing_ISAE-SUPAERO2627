function out = scheduledWeightsConfig()

% Output weights: cost of tracking error on each output [x y z vx vy vz qx qy qz p q r].
%   qx qy qz are the quaternion vector part.
%   1 = tracked normally, 0 = not tracked.
% As in every opt, the relative wheights are what matters not their abs value

out.out  = [0.3 0.3 0.3  1 1 1  0   0   0  0   0   0  ; % APPROACG
            1   1   1    1 1 1  0   0   0  0   0   0  ; % CLOSEUP
            3   3   1    2 2 2  0   0   0  0.1 0.1 0.1; % LANDING
            5   5   3    3 3 3  0.5 0.5 0  0.1 0.1 0.1]; % FLARE

out.rate = [0.2  0.2  0.2  0.2 ;
            0.1  0.1  0.1  0.1 ;
            0.1  0.1  0.1  0.1 ;
            0.05 0.05 0.05 0.05];

end
