function test_aero_limits()
%TEST_AERO_LIMITS Basic sanity checks for geometry/aero pipeline.

thisDir = fileparts(mfilename('fullpath'));
stlPath = fullfile(thisDir, '..', '..', '3d', 'BorosRocket.stl');
assert(exist(stlPath, 'file') == 2, 'STL not found: %s', stlPath);

geom = geometry_loader(stlPath, 1e-3, 0.90);
assert(geom.length_m > 0.10 && geom.length_m < 2.0, 'Unexpected rocket length');
assert(geom.body_diameter_m > 0.01 && geom.body_diameter_m < 0.20, 'Unexpected body diameter');
assert(geom.frontal_area_m2 > 1e-4, 'Frontal area too small');

env = struct('g', 9.80665, 'rho0', 1.225, 'mu0', 1.81e-5, 'a0', 340.3);
aero = aero_coefficients((10:10:200)', env, geom, struct());
assert(all(aero.Cd > 0.03), 'Cd must stay positive');
assert(all(aero.Cd < 2.5), 'Cd unexpectedly high');
assert(max(aero.Mach) < 1.0, 'Sweep exceeded intended subsonic region');

mass = mass_properties(geom, 0.080, struct('motorTotalMass_kg', 0.0241, 'propMass_kg', 0.0122));
stab = stability_analysis(geom, mass);
assert(stab.margin_loaded_cal > 0.0, 'Negative static margin');

cd100 = interp1(aero.v_mps, aero.Cd, 100, 'linear', 'extrap');
fprintf('PASS test_aero_limits | L=%.3f m D=%.3f m Cd@100=%.3f\n', ...
    geom.length_m, geom.body_diameter_m, cd100);
end
