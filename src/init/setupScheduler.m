function out = setupScheduler()

scheduler.radius1_in =  20;
scheduler.radius1_out = 20.1;

scheduler.radius2_in = 5;
scheduler.radius2_out = 6.5;   % wide hysteresis: avoids LANDING<->CLOSEUP chattering

scheduler.radius3_in = 1;
scheduler.radius3_out = Inf;   % FLARE is irreversible (never goes back to LANDING)

out = scheduler;

end





