function mass = mass_properties(geom, totalMassKg, engine)
%MASS_PROPERTIES Estimate mass/inertia and CG for the assembled rocket.
%
% Inputs:
%   geom        - struct from geometry_loader
%   totalMassKg - launch mass (rocket + motor loaded)
%   engine      - struct with fields:
%                 .motorTotalMass_kg, .propMass_kg

if nargin < 2 || isempty(totalMassKg)
    totalMassKg = 0.080;
end
if nargin < 3 || isempty(engine)
    engine = struct();
end
if ~isfield(engine, 'motorTotalMass_kg'); engine.motorTotalMass_kg = 0.0241; end
if ~isfield(engine, 'propMass_kg'); engine.propMass_kg = 0.0122; end

L = max(geom.length_m, 1e-3);
D = max(geom.body_diameter_m, 1e-3);
R = D / 2;

dryMass = totalMassKg - engine.propMass_kg;
if dryMass <= 0
    error('Invalid mass set: dry mass <= 0');
end

Ixx = 0.5 * totalMassKg * R^2;
Iyy = (1/12) * totalMassKg * (3*R^2 + L^2);
Izz = Iyy;

% Engineering estimate for model rockets with aft motor.
cgLoaded = 0.56 * L;
cgDry = 0.53 * L;

mass = struct();
mass.total_liftoff_kg = totalMassKg;
mass.dry_kg = dryMass;
mass.motor_total_kg = engine.motorTotalMass_kg;
mass.prop_kg = engine.propMass_kg;
mass.motor_case_kg = max(engine.motorTotalMass_kg - engine.propMass_kg, 0);
mass.cg_loaded_m = cgLoaded;
mass.cg_dry_m = cgDry;
mass.inertia_loaded_kgm2 = [Ixx, Iyy, Izz];
mass.inertia_dry_kgm2 = (dryMass / totalMassKg) * [Ixx, Iyy, Izz];
mass.mass_flow_kgps = engine.propMass_kg / 1.86; % nominal C6 burn window
end
