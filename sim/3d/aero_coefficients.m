function aero = aero_coefficients(vSweep_mps, env, geom, opts)
%AERO_COEFFICIENTS Compute Cd table with Re/Mach corrections.

if nargin < 1 || isempty(vSweep_mps)
    vSweep_mps = (10:10:200)';
end
if nargin < 2 || isempty(env)
    env = struct('g', 9.80665, 'rho0', 1.225, 'mu0', 1.81e-5, 'a0', 340.3);
end
if nargin < 3
    error('geom is required');
end
if nargin < 4 || isempty(opts)
    opts = struct();
end

if ~isfield(opts, 'roughness_m'); opts.roughness_m = 2e-5; end
if ~isfield(opts, 'conservative'); opts.conservative = true; end
if ~isfield(opts, 'flightCdScale'); opts.flightCdScale = 1.10; end
if ~isfield(opts, 'transonicScale'); opts.transonicScale = 1.00; end
if ~isfield(opts, 'minCd'); opts.minCd = 0.05; end

vSweep_mps = vSweep_mps(:);
n = numel(vSweep_mps);
Cd = zeros(n,1);
Mach = zeros(n,1);
Re = zeros(n,1);
CdFric = zeros(n,1);
CdBase = zeros(n,1);
CdWave = zeros(n,1);

D = max(geom.body_diameter_m, 1e-5);
Aref = max(geom.frontal_area_m2, 1e-9);
Awet = max(geom.wetted_area_m2, Aref);
L = max(geom.length_m, 1e-5);

for i = 1:n
    v = abs(vSweep_mps(i));
    Mach(i) = v / max(env.a0, 1e-6);
    Re(i) = env.rho0 * v * D / max(env.mu0, 1e-8);

    if Re(i) < 5e5
        Cf = 1.328 / sqrt(max(Re(i), 1));
    else
        Cf = max(0.074 / (Re(i)^0.2) - 1742 / Re(i), 0);
    end
    CfRough = 0.032 * (opts.roughness_m / L)^0.2;
    Cf = max(Cf, CfRough);

    CdFric(i) = Cf * (Awet / Aref);
    CdBase(i) = 0.12 + 0.13 * (Mach(i)^2) / (1 + Mach(i)^2);
    if Mach(i) < 0.8
        CdWave(i) = 0;
    elseif Mach(i) < 1.2
        p = (Mach(i) - 0.8) / 0.4;
        CdWave(i) = 0.22 * sin(pi * p)^2 * opts.transonicScale;
    else
        CdWave(i) = 0.10 / sqrt(Mach(i));
    end

    Cd(i) = CdFric(i) + CdBase(i) + CdWave(i);
end

if opts.conservative
    Cd = 1.08 * Cd;
end
Cd = max(opts.minCd, Cd * opts.flightCdScale);

aero = struct();
aero.v_mps = vSweep_mps;
aero.Cd = Cd;
aero.Mach = Mach;
aero.Re = Re;
aero.Cd_friction = CdFric;
aero.Cd_base = CdBase;
aero.Cd_wave = CdWave;
aero.opts = opts;
end
