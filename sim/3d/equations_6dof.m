function ds = equations_6dof(t, s, p)
%EQUATIONS_6DOF Simplified planar 6-state dynamics:
% s = [x z vx vz theta q]'
%
% Inputs in params struct p:
%   p.env.g
%   p.engine.t / p.engine.F
%   p.mass.dry_kg / p.mass.prop_kg / p.burnTime_s
%   p.aero.CdOfV(v), p.aero.Aref
%   p.windOfH(z)
%   p.kAlpha, p.kQ, p.Iyy
%   p.chute.enabled, p.chute.deployTime_s, p.chute.inflate_s, p.chute.Cd, p.chute.area_m2

x = s(1); %#ok<NASGU>
z = max(s(2), 0);
vx = s(3);
vz = s(4);
theta = s(5);
q = s(6);

burnFrac = min(max(t / max(p.burnTime_s, 1e-6), 0), 1);
mass = p.mass.dry_kg + p.mass.prop_kg * (1 - burnFrac);
thrust = interp1(p.engine.t, p.engine.F, t, 'linear', 0);

if nargin(p.windOfH) == 2
    wind = p.windOfH(z, t);
else
    wind = p.windOfH(z);
end
vRel = [vx - wind; vz];
vAir = max(norm(vRel), 1e-9);
rho = p.rhoOfH(z);
Cd = p.aero.CdOfV(vAir);

drag = -0.5 * rho * Cd * p.aero.Aref * vAir * vRel;

chuteDrag = [0; 0];
if p.chute.enabled && (t >= p.chute.deployTime_s)
    alpha = min((t - p.chute.deployTime_s) / max(p.chute.inflate_s, 1e-3), 1);
    chuteDrag = -0.5 * rho * p.chute.Cd * p.chute.area_m2 * alpha * vAir * vRel;
end

u = [cos(theta); sin(theta)];
fThrust = thrust * u;
fGrav = [0; -mass * p.env.g];
fTot = fThrust + drag + chuteDrag + fGrav;

ax = fTot(1) / mass;
az = fTot(2) / mass;

if vAir > 1e-3
    gamma = atan2(vRel(2), vRel(1));
else
    gamma = theta;
end
alpha = i_wrap_to_pi(gamma - theta);
mAero = -p.kAlpha * alpha - p.kQ * q;
qdot = mAero / max(p.Iyy, 1e-6);

ds = zeros(6,1);
ds(1) = vx;
ds(2) = vz;
ds(3) = ax;
ds(4) = az;
ds(5) = q;
ds(6) = qdot;
end

function a = i_wrap_to_pi(a)
a = mod(a + pi, 2*pi) - pi;
end
