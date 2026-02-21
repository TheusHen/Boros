function drag = drag_model(geom, env, aeroOpts)
%DRAG_MODEL Build drag lookup tables from geometry and environment.

if nargin < 2 || isempty(env)
    env = struct('rho0', 1.225, 'a0', 340.3, 'mu0', 1.81e-5);
end
if nargin < 3 || isempty(aeroOpts)
    aeroOpts = struct();
end
if ~isfield(aeroOpts, 'vSweep_mps')
    aeroOpts.vSweep_mps = (10:10:200)';
end

aero = aero_coefficients(aeroOpts.vSweep_mps, env, geom, aeroOpts);
q = 0.5 * env.rho0 * (aero.v_mps .^ 2);
D = q .* aero.Cd * geom.frontal_area_m2;

drag = struct();
drag.v_mps = aero.v_mps;
drag.Cd = aero.Cd;
drag.q_pa = q;
drag.D_n = D;
drag.Mach = aero.Mach;
drag.Re = aero.Re;
drag.CdComponents = struct( ...
    'friction', aero.Cd_friction, ...
    'base', aero.Cd_base, ...
    'wave', aero.Cd_wave);
drag.CdOfV = @(v) interp1(drag.v_mps, drag.Cd, abs(v), 'linear', 'extrap');
drag.DragOfV = @(v, rho) 0.5 * rho .* (v.^2) .* drag.CdOfV(v) * geom.frontal_area_m2;
end
