function out = scheduledLimitsConfig()

%              min  max
out.thrust = [ -3   3 ;   % APPROACH
               -3   3 ;   % CLOSEUP
               -3   3 ;   % LANDING
               -3   3 ];  % FLARE

out.torque = [ -5   5 ;
               -5   5 ;
               -5   5 ;
               -5   5 ];

%              [deg]
out.tilt   = [ Inf ;
               Inf ;
               Inf ;
               Inf ];

%              |v| max [m/s], Inf = free
out.vel    = [ Inf Inf Inf ;
               Inf Inf Inf ;
               Inf Inf Inf ;
               Inf Inf Inf ];

out.posMin = [ -Inf -Inf -20+15 ;
               -Inf -Inf -20+3 ;
               -Inf -Inf -20+1 ;
               -Inf -Inf -20-1 ];

out.posMax = [  Inf  Inf  Inf ;
                Inf  Inf  Inf ;
                Inf  Inf  Inf ;
                Inf  Inf  Inf ];

out.angRate = [ Inf Inf Inf ;
                Inf Inf Inf ;
                Inf Inf Inf ;
                Inf Inf Inf ];

out.ecr    = [ 1 ;
               1 ;
               1 ;
               1 ];

end
