% BOROS aerodynamic and trajectory report (MATLAB)
% This script uses STL geometry + C6 thrust curve + wind + recovery.

clear; clc;

thisDir = fileparts(mfilename('fullpath'));
geomCandidate = fullfile(thisDir, '..', '..', '3d', 'BorosRocket.stl');
if isfile(geomCandidate)
    geomFile = string(geomCandidate);
else
    geomFile = "BorosRocket.stl";
end

unitScale = 1e-3;               % STL in mm -> m
bodyQuantile = 0.90;
outDir = fullfile(thisDir, 'out');
if ~exist(outDir, 'dir'); mkdir(outDir); end

% Environment
env = struct();
env.g = 9.80665;
env.rho0 = 1.225;
env.mu0 = 1.81e-5;
env.a0 = 340.3;
env.H = 8500;

% Aerodynamic model
aeroOpts = struct();
aeroOpts.roughness_m = 2e-5;
aeroOpts.vSweep_mps = (10:10:200)';
aeroOpts.conservative = true;
aeroOpts.flightCdScale = 1.10;
aeroOpts.transonicScale = 1.00;
aeroOpts.minCd = 0.05;

% Mass/motor (user request: total 80g + C6-5)
totalMassKg = 0.080;
engine = struct();
engine.delay_s = 5.0;
engine.motorTotalMass_kg = 0.0241;
engine.propMass_kg = 0.0122;
engine.totalImpulse_Ns = 10.0;

% C6 curve shape (RASP/NAR), scaled to target impulse.
engine.t = [ ...
    0.000 0.031 0.092 0.139 0.192 0.209 0.231 0.248 0.292 0.370 ...
    0.475 0.671 0.702 0.723 0.850 1.063 1.211 1.242 1.303 1.468 ...
    1.656 1.821 1.834 1.847 1.860 ]';
engine.F = [ ...
    0.000 0.946 4.826 9.936 14.090 11.446 7.381 6.151 5.489 4.921 ...
    4.448 4.258 4.542 4.164 4.448 4.353 4.353 4.069 4.258 4.353 ...
    4.448 4.448 2.933 1.325 0.000 ]';
engine.F = engine.F * (engine.totalImpulse_Ns / trapz(engine.t, engine.F));

% Launch
launch = struct();
launch.angle_deg = 88;
launch.rail_m = 1.0;
launch.v0_mps = 0.0;
launch.z0_m = 0.0;

% Wind
wind = struct();
wind.profile = "linear";      % constant | linear | piecewise
wind.constant_mps = 0.0;
wind.wind0_mps = 1.0;
wind.windTop_mps = 4.0;
wind.hTop_m = 300.0;
wind.h_m = [0 50 100 200 300 400]';
wind.wind_mps = [1 2 2.5 3.5 4 4.5]';
wind.gust_sigma_mps = 0.5;
wind.gust_freq_hz = 0.22;
wind.gust_phase = 1.4;

% Recovery
recovery = struct();
recovery.enabled = true;
recovery.mode = "time";
recovery.Cd_chute = 1.35;
recovery.auto_size = true;
recovery.v_target_mps = 3.0;
recovery.diam_m = 0.35;
recovery.deploy_extra_s = 0.0;
recovery.inflation_time_s = 0.6;

geom = geometry_loader(geomFile, unitScale, bodyQuantile);
mass = mass_properties(geom, totalMassKg, engine);
drag = drag_model(geom, env, aeroOpts);
stab = stability_analysis(geom, mass);

if recovery.auto_size
    chuteArea = 2 * mass.dry_kg * env.g / (env.rho0 * recovery.Cd_chute * recovery.v_target_mps^2);
    recovery.diam_m = sqrt(4 * chuteArea / pi);
else
    chuteArea = pi * (recovery.diam_m/2)^2;
end

cfg = struct();
cfg.env = env;
cfg.engine = engine;
cfg.mass = mass;
cfg.aero = struct();
cfg.aero.CdOfV = drag.CdOfV;
cfg.aero.Aref = geom.frontal_area_m2;
cfg.launch = launch;
cfg.att = struct('kAlpha', 0.08, 'kQ', 0.03);
cfg.tMax_s = 180;
cfg.rhoOfH = @(h) env.rho0 .* exp(-max(h,0) / env.H);
cfg.windOfH = @(h, t) i_wind_profile(h, t, wind);

cfg.recovery = struct();
cfg.recovery.enabled = recovery.enabled;
cfg.recovery.Cd = recovery.Cd_chute;
cfg.recovery.area_m2 = chuteArea;
cfg.recovery.inflate_s = recovery.inflation_time_s;
cfg.recovery.deployTime_s = engine.t(end) + engine.delay_s + recovery.deploy_extra_s;

sim = attitude_sim(cfg);

% Outputs
cdTable = table(drag.v_mps, drag.Cd, drag.Mach, drag.Re, drag.D_n, ...
    'VariableNames', {'speed_mps','Cd','Mach','Re','Drag_N'});
writetable(cdTable, fullfile(outDir, 'cd_table_matlab.csv'));

trajTable = table(sim.t, sim.state(:,1), sim.state(:,2), sim.state(:,3), sim.state(:,4), ...
    sim.state(:,5), sim.state(:,6), ...
    'VariableNames', {'time_s','x_m','z_m','vx_mps','vz_mps','theta_rad','q_rps'});
writetable(trajTable, fullfile(outDir, 'trajectory_matlab.csv'));

report = struct();
report.geometry = rmfield(geom, {'faces','vertices'});
report.mass = mass;
report.stability = stab;
report.summary = sim.summary;
report.recovery = recovery;
report.engine = struct('burnTime_s', engine.t(end), 'totalImpulse_Ns', trapz(engine.t, engine.F));
report.outputs = struct('cd_table', 'cd_table_matlab.csv', 'trajectory', 'trajectory_matlab.csv');

save(fullfile(outDir, 'report_matlab.mat'), 'report', 'sim', 'cfg', 'drag', 'geom', 'mass', 'stab');

jsonText = jsonencode(report);
fid = fopen(fullfile(outDir, 'report_matlab.json'), 'w');
fwrite(fid, jsonText, 'char');
fclose(fid);

fprintf('=== BOROS MATLAB Simulation ===\n');
fprintf('STL: %s\n', geom.file);
fprintf('Length: %.3f m | Body D: %.1f mm\n', geom.length_m, 1000*geom.body_diameter_m);
fprintf('Apogee: %.1f m at %.2f s\n', sim.summary.apogee_m, sim.summary.apogee_t_s);
fprintf('Drift: %.1f m | Flight time: %.2f s\n', sim.summary.drift_m, sim.summary.flight_time_s);
fprintf('Parachute diameter: %.3f m\n', recovery.diam_m);
fprintf('Outputs: %s\n', outDir);

function w = i_wind_profile(h, t, wind)
h = max(h, 0);
switch lower(string(wind.profile))
    case "constant"
        w0 = wind.constant_mps;
    case "linear"
        a = min(max(h / max(wind.hTop_m, 1), 0), 1);
        w0 = wind.wind0_mps + a * (wind.windTop_mps - wind.wind0_mps);
    otherwise
        w0 = interp1(wind.h_m, wind.wind_mps, h, 'linear', 'extrap');
end
w = w0 + wind.gust_sigma_mps * sin(2*pi*wind.gust_freq_hz*t + wind.gust_phase);
end
