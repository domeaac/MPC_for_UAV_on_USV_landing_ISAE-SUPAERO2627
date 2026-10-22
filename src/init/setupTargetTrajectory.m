function out = setupTargetTrajectory()
% target.velocity_profile_x = @(t) 1; 
% target.velocity_profile_y = @(t) 1; 
% target.velocity_profile_z = @(t) 1; 

out.R = 5; 
out.w = 0.2; 

out.target.velocity_profile_x = @(t) -out.R * out.w * sin(out.w * t);
out.target.velocity_profile_y = @(t) out.R * out.w * cos(out.w * t);
out.target.velocity_profile_z = @(t) 0; 

% EITHER
out.target.x0Type = "set";
out.target.x0 = [2; 3; 5]; % m

% OR
%x0_target_type = "random"
out.target.staticAttitudeFlag = 0;

out.target.initialDCM = [1, 0, 0;
    0, 1, 0;
    0, 0, 1];

out.target.attitude_euler = @(t) [0; 0; pi/2] + [0; 0; out.w * t];

end