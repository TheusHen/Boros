function run_ci_sim()
%RUN_CI_SIM CI-friendly 3D simulation runner (MATLAB + Octave compatible).

thisDir = fileparts(mfilename('fullpath'));
outDir = fullfile(thisDir, 'out');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

stlPath = fullfile(thisDir, '..', '..', '3d', 'BorosRocket.stl');
if ~exist(stlPath, 'file')
    error('STL not found: %s', stlPath);
end

% Baseline parameters (user-defined mission profile).
env = struct('g', 9.80665, 'rho0', 1.225, 'mu0', 1.81e-5, 'a0', 340.3, 'H', 8500);
aeroOpts = struct( ...
    'roughness_m', 2e-5, ...
    'vSweep_mps', (10:10:200)', ...
    'conservative', true, ...
    'flightCdScale', 1.10, ...
    'transonicScale', 1.00, ...
    'minCd', 0.05);

engine = struct();
engine.delay_s = 5.0;
engine.motorTotalMass_kg = 0.0241;
engine.propMass_kg = 0.0122;
engine.totalImpulse_Ns = 10.0;
engine.t = [ ...
    0.000 0.031 0.092 0.139 0.192 0.209 0.231 0.248 0.292 0.370 ...
    0.475 0.671 0.702 0.723 0.850 1.063 1.211 1.242 1.303 1.468 ...
    1.656 1.821 1.834 1.847 1.860 ]';
engine.F = [ ...
    0.000 0.946 4.826 9.936 14.090 11.446 7.381 6.151 5.489 4.921 ...
    4.448 4.258 4.542 4.164 4.448 4.353 4.353 4.069 4.258 4.353 ...
    4.448 4.448 2.933 1.325 0.000 ]';
engine.F = engine.F * (engine.totalImpulse_Ns / trapz(engine.t, engine.F));

launch = struct('angle_deg', 88, 'rail_m', 1.0, 'v0_mps', 0.0, 'z0_m', 0.0);
wind = struct( ...
    'profile', 'linear', ...
    'constant_mps', 0.0, ...
    'wind0_mps', 1.0, ...
    'windTop_mps', 4.0, ...
    'hTop_m', 300.0, ...
    'h_m', [0 50 100 200 300 400]', ...
    'wind_mps', [1 2 2.5 3.5 4 4.5]');

recovery = struct( ...
    'enabled', true, ...
    'Cd_chute', 1.35, ...
    'v_target_mps', 3.0, ...
    'deploy_extra_s', 0.0, ...
    'inflation_time_s', 0.6);

geom = geometry_loader(stlPath, 1e-3, 0.90);
mass = mass_properties(geom, 0.080, engine);
drag = drag_model(geom, env, aeroOpts);
stab = stability_analysis(geom, mass);

chuteArea = 2 * mass.dry_kg * env.g / (env.rho0 * recovery.Cd_chute * recovery.v_target_mps^2);
chuteDiam = sqrt(4 * chuteArea / pi);

cfg = struct();
cfg.env = env;
cfg.engine = engine;
cfg.mass = mass;
cfg.aero = struct('CdOfV', drag.CdOfV, 'Aref', geom.frontal_area_m2);
cfg.launch = launch;
cfg.att = struct('kAlpha', 0.08, 'kQ', 0.03);
cfg.tMax_s = 180;
cfg.rhoOfH = @(h) env.rho0 .* exp(-max(h,0) / env.H);
cfg.windOfH = @(h, t) i_wind_profile(h, t, wind);
cfg.recovery = struct( ...
    'enabled', recovery.enabled, ...
    'Cd', recovery.Cd_chute, ...
    'area_m2', chuteArea, ...
    'inflate_s', recovery.inflation_time_s, ...
    'deployTime_s', engine.t(end) + engine.delay_s + recovery.deploy_extra_s);

sim = attitude_sim(cfg);

% Write artifacts with pure fprintf (Octave-safe).
i_write_csv(fullfile(outDir, 'cd_table_matlab.csv'), ...
    {'speed_mps','Cd','Mach','Re','Drag_N'}, ...
    [drag.v_mps, drag.Cd, drag.Mach, drag.Re, drag.D_n]);

i_write_csv(fullfile(outDir, 'trajectory_matlab.csv'), ...
    {'time_s','x_m','z_m','vx_mps','vz_mps','theta_rad','q_rps'}, ...
    [sim.t, sim.state(:,1), sim.state(:,2), sim.state(:,3), sim.state(:,4), sim.state(:,5), sim.state(:,6)]);

summaryPath = fullfile(outDir, 'report_matlab_summary.txt');
fid = fopen(summaryPath, 'w');
if fid < 0
    error('Unable to write summary file: %s', summaryPath);
end
fprintf(fid, 'stl=%s\n', geom.file);
fprintf(fid, 'length_m=%.6f\n', geom.length_m);
fprintf(fid, 'body_diameter_m=%.6f\n', geom.body_diameter_m);
fprintf(fid, 'apogee_m=%.6f\n', sim.summary.apogee_m);
fprintf(fid, 'apogee_t_s=%.6f\n', sim.summary.apogee_t_s);
fprintf(fid, 'drift_m=%.6f\n', sim.summary.drift_m);
fprintf(fid, 'flight_time_s=%.6f\n', sim.summary.flight_time_s);
fprintf(fid, 'landing_speed_mps=%.6f\n', sim.summary.landing_speed_mps);
fprintf(fid, 'chute_diameter_m=%.6f\n', chuteDiam);
fprintf(fid, 'stability_margin_loaded_cal=%.6f\n', stab.margin_loaded_cal);
fprintf(fid, 'stability_margin_dry_cal=%.6f\n', stab.margin_dry_cal);
fclose(fid);

if exist('jsonencode', 'builtin') || exist('jsonencode', 'file')
    report = struct();
    report.summary = sim.summary;
    report.geometry = rmfield(geom, {'faces','vertices'});
    report.mass = mass;
    report.stability = stab;
    report.recovery = struct('chute_diameter_m', chuteDiam, 'Cd_chute', recovery.Cd_chute);
    txt = jsonencode(report);
    fid = fopen(fullfile(outDir, 'report_matlab.json'), 'w');
    if fid >= 0
        fwrite(fid, txt, 'char');
        fclose(fid);
    end
end

fprintf('=== run_ci_sim completed (%s) ===\n', i_backend_name());
fprintf('Apogee: %.1f m | Drift: %.1f m | Flight: %.2f s\n', ...
    sim.summary.apogee_m, sim.summary.drift_m, sim.summary.flight_time_s);
fprintf('Outputs: %s\n', outDir);
end

function w = i_wind_profile(h, t, wind)
h = max(h, 0);
switch lower(char(wind.profile))
    case 'constant'
        w0 = wind.constant_mps;
    case 'linear'
        a = min(max(h / max(wind.hTop_m, 1), 0), 1);
        w0 = wind.wind0_mps + a * (wind.windTop_mps - wind.wind0_mps);
    otherwise
        w0 = interp1(wind.h_m, wind.wind_mps, h, 'linear', 'extrap');
end
w = w0 + 0.4 * sin(2*pi*0.18*t + 1.1);
end

function i_write_csv(path, headers, data)
fid = fopen(path, 'w');
if fid < 0
    error('Unable to write CSV: %s', path);
end
for i = 1:numel(headers)
    if i > 1, fprintf(fid, ','); end
    fprintf(fid, '%s', headers{i});
end
fprintf(fid, '\n');
for r = 1:size(data,1)
    for c = 1:size(data,2)
        if c > 1, fprintf(fid, ','); end
        fprintf(fid, '%.9g', data(r,c));
    end
    fprintf(fid, '\n');
end
fclose(fid);
end

function name = i_backend_name()
if exist('OCTAVE_VERSION', 'builtin')
    name = ['Octave ' OCTAVE_VERSION];
else
    name = ['MATLAB ' version];
end
end
