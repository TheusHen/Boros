function stab = stability_analysis(geom, mass, varargin)
%STABILITY_ANALYSIS Quick static stability estimate (calibers).
%
% Optional name-value:
%   'cpFrac', default 0.63  (CP location as fraction of length)

p = inputParser;
addParameter(p, 'cpFrac', 0.63, @(x) isnumeric(x) && isscalar(x));
parse(p, varargin{:});

L = max(geom.length_m, 1e-6);
D = max(geom.body_diameter_m, 1e-6);
cp = p.Results.cpFrac * L;
cgLoaded = mass.cg_loaded_m;
cgDry = mass.cg_dry_m;

marginLoaded = (cp - cgLoaded) / D;
marginDry = (cp - cgDry) / D;

stab = struct();
stab.cp_m = cp;
stab.cg_loaded_m = cgLoaded;
stab.cg_dry_m = cgDry;
stab.margin_loaded_cal = marginLoaded;
stab.margin_dry_cal = marginDry;
stab.is_stable_loaded = marginLoaded >= 1.0;
stab.is_stable_dry = marginDry >= 1.0;
stab.note = "Regra pratica: margem estatica entre 1.0 e 2.5 calibres.";
end
