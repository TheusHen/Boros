function geom = geometry_loader(geomFile, unitScale, bodyQuantile)
%GEOMETRY_LOADER Load STL and estimate body geometry robustly.
%   geom = GEOMETRY_LOADER(geomFile, unitScale, bodyQuantile)
%
% Inputs:
%   geomFile      - STL path
%   unitScale     - STL units to meters (e.g. 1e-3 for mm)
%   bodyQuantile  - radius quantile used to reject fins/outliers

if nargin < 2 || isempty(unitScale)
    unitScale = 1e-3;
end
if nargin < 3 || isempty(bodyQuantile)
    bodyQuantile = 0.90;
end

[F, V] = i_read_stl_mesh(geomFile);
V = V .* unitScale;

vMin = min(V, [], 1);
vMax = max(V, [], 1);
vCtr = mean(V, 1);
V0 = V - vCtr;

C = (V0' * V0) / max(size(V0,1) - 1, 1);
[eigVec, eigVal] = eig(C);
[~, k] = max(diag(eigVal));
axisVec = eigVec(:, k)';
axisVec = axisVec ./ max(norm(axisVec), eps);

axial = V0 * axisVec';
length_m = max(axial) - min(axial);

radialVec = V0 - axial * axisVec;
radii = sqrt(sum(radialVec.^2, 2));
q = min(max(bodyQuantile, 0.50), 0.99);
bodyRadius = i_quantile(radii, q);
maxRadius = max(radii);

bodyDiameter = 2 * bodyRadius;
maxDiameter = 2 * maxRadius;
frontalArea = pi * bodyRadius^2;

e1 = V(F(:,2), :) - V(F(:,1), :);
e2 = V(F(:,3), :) - V(F(:,1), :);
triArea = 0.5 * sqrt(sum(cross(e1, e2, 2).^2, 2));
wettedArea = sum(triArea);

volSigned = dot(V(F(:,1), :), cross(V(F(:,2), :), V(F(:,3), :), 2), 2) / 6;
volume = abs(sum(volSigned));

geom = struct();
geom.file = char(geomFile);
geom.length_m = length_m;
geom.body_diameter_m = bodyDiameter;
geom.max_diameter_m = maxDiameter;
geom.frontal_area_m2 = frontalArea;
geom.wetted_area_m2 = wettedArea;
geom.volume_m3 = volume;
geom.axis = axisVec;
geom.bounds_min_m = vMin;
geom.bounds_max_m = vMax;
geom.faces = F;
geom.vertices = V;
end

function [F, V] = i_read_stl_mesh(filePath)
% Try built-in stlread first, fallback to lightweight parser.
if exist('stlread', 'file') == 2
    try
        obj = stlread(filePath);
        if isa(obj, 'triangulation')
            F = obj.ConnectivityList;
            V = obj.Points;
            return;
        end
    catch
        % fallback parser below
    end
end

fid = fopen(filePath, 'r');
if fid < 0
    error('Unable to open STL file: %s', filePath);
end
cleanup = onCleanup(@() fclose(fid));

hdr = fread(fid, 80, '*uint8');
if numel(hdr) < 80
    error('Invalid STL header.');
end
nTri = fread(fid, 1, 'uint32');
info = dir(filePath);
expected = 84 + 50 * double(nTri);

fseek(fid, 0, 'bof');
if expected == info.bytes && nTri > 0
    fseek(fid, 84, 'bof');
    V = zeros(double(nTri) * 3, 3);
    F = zeros(double(nTri), 3);
    for i = 1:double(nTri)
        fread(fid, 3, 'float32'); % normal
        v1 = fread(fid, 3, 'float32')';
        v2 = fread(fid, 3, 'float32')';
        v3 = fread(fid, 3, 'float32')';
        fread(fid, 1, 'uint16'); % attribute
        idx = 3*(i-1) + (1:3);
        V(idx, :) = [v1; v2; v3];
        F(i, :) = idx;
    end
    return;
end

% ASCII STL fallback
fseek(fid, 0, 'bof');
txt = fscanf(fid, '%c');
verts = zeros(0,3);
lines = regexp(txt, '\r\n|\n|\r', 'split');
for i = 1:numel(lines)
    s = strtrim(lines{i});
    if numel(s) >= 6 && strcmpi(s(1:6), 'vertex')
        vals = sscanf(s, 'vertex %f %f %f');
        if numel(vals) == 3
            verts(end+1, :) = vals(:)'; %#ok<AGROW>
        end
    end
end

if mod(size(verts,1), 3) ~= 0 || isempty(verts)
    error('Failed to parse STL (binary or ASCII): %s', filePath);
end

V = verts;
nTri = size(verts,1) / 3;
F = reshape(1:(3*nTri), 3, [])';
end

function qv = i_quantile(x, q)
x = sort(x(:));
if isempty(x)
    qv = 0;
    return;
end
q = min(max(q, 0), 1);
pos = 1 + (numel(x) - 1) * q;
i0 = floor(pos);
i1 = ceil(pos);
if i0 == i1
    qv = x(i0);
else
    t = pos - i0;
    qv = x(i0) * (1 - t) + x(i1) * t;
end
end
