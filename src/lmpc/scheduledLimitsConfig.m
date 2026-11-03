function out = scheduledLimitsConfig()

%              min  max
out.thrust = [ -3   3 ;   % APPROACH
               -3   3 ;   % CLOSEUP
               -2   2 ;   % LANDING
               -1   1 ];  % FLARE

out.torque = [ -5   5 ;
               -5   5 ;
               -3   3 ;
               -2   2 ];

%              [deg]
out.tilt   = [ 20 ;
               15 ;
               10 ;
                5 ];

%              |v| max [m/s], Inf = free
out.vel    = [ Inf Inf Inf ;
                5   5   2  ;
                2   2   1  ;
                1   1  0.5 ];

end
