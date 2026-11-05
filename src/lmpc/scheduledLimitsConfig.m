function out = scheduledLimitsConfig()

val = -20;

%              min  max
out.thrust = [ -3   3 ;   % APPROACH
               -3   3 ;   % CLOSEUP
               -3   3 ;   % LANDING
               -3   3 ];  % FLARE

out.torque = [ -5   5 ;
               -5   5 ;
               -5   5 ;
               -5   5 ]*1/5;

%              [deg]
out.tilt   = [ 20 ;
               15 ;
               10 ;
               5 ];

%              |v| max [m/s], Inf = free
out.vel    = [ 5   5   5   ;
               3   3   1.5 ;
               1.5 1.5 0.8 ;
               0.5 0.5 0.4 ];

out.posMin = [ -inf -inf -inf ;
               val-25 val-25 val+4 ;
               val-6 val-6 val+0.5 ;
               val-1 val-1 val ];

out.posMax = [  inf  inf  inf ;
                val+25  val+25  val+25 ;
                val+6  val+6  val+6 ;
                val+2  val+2  val+2 ];

%               [deg/s] - tighter values are violated anyway and only add conflicts
out.angRate = [ 60 60 60 ;
                45 45 45 ;
                30 30 30 ;
                30 30 30 ]*pi/180;

out.ecr  = [  1    0.2  0.01  0.2 ;   % APPROACH
              1    0.2  0.01  0.2 ;   % CLOSEUP
              1    0.2  0.01  0.2 ;   % LANDING
              1    0.2  0.01  0.2 ];  % FLARE
end
