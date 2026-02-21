function sim = attitude_sim(cfg)
%ATTITUDE_SIM Integrate simplified 6-state trajectory + attitude model.

if nargin < 1
    error('cfg struct is required');
end

params = struct();
params.env = cfg.env;
params.engine = cfg.engine;
params.mass = cfg.mass;
params.burnTime_s = cfg.engine.t(end);
params.aero = cfg.aero;
params.kAlpha = cfg.att.kAlpha;
params.kQ = cfg.att.kQ;
params.Iyy = cfg.mass.inertia_loaded_kgm2(2);
params.chute = cfg.recovery;
params.rhoOfH = cfg.rhoOfH;
params.windOfH = cfg.windOfH;

ang = deg2rad(cfg.launch.angle_deg);
s0 = [
    0; ...
    max(cfg.launch.z0_m, 0); ...
    cfg.launch.v0_mps * cos(ang); ...
    cfg.launch.v0_mps * sin(ang); ...
    ang; ...
    0];

tspan = [0, cfg.tMax_s];
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'Events', @i_event_ground);
[t, s] = ode45(@(tt, ss) equations_6dof(tt, ss, params), tspan, s0, opts);

z = s(:,2);
v = hypot(s(:,3), s(:,4));
[apogee_m, kApo] = max(z);
apogee_t = t(kApo);

sim = struct();
sim.t = t;
sim.state = s;
sim.summary = struct( ...
    'apogee_m', apogee_m, ...
    'apogee_t_s', apogee_t, ...
    'flight_time_s', t(end), ...
    'landing_speed_mps', v(end), ...
    'drift_m', s(end,1), ...
    'max_speed_mps', max(v));
end

function [value, isterminal, direction] = i_event_ground(t, s)
if t < 0.25
    value = 1;
else
    value = s(2);
end
isterminal = 1;
direction = -1;
end
