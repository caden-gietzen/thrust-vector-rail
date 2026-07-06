function idp = loadIdentifiedParams()
%LOADIDENTIFIEDPARAMS  Load the canonical identified-parameter store.
%   idp = LOADIDENTIFIEDPARAMS() returns the identified-parameter struct read
%   from data/processed/identified_parameters/identified_parameters.mat. If the
%   .mat is missing or older than the JSON source, it is rebuilt automatically
%   via buildIdentifiedParams so callers always get the current values.
%
%   Leaves are the full {value, units, sigma, ci, source, date} structs, e.g.
%       idp.servo.K_theta.value   -> 0.001618 (rad/us)
%       idp.thrust.static_map.coeffs_high_to_low
%   Nominal means feed params.m; the sigma/ci fields feed the later uncertainty
%   sweeps. See buildIdentifiedParams.m for how the store is built.

    here = fileparts(mfilename('fullpath'));
    repoRoot = fileparts(fileparts(here)); % analysis/utils -> repo root
    dataDir = fullfile(repoRoot, 'data', 'processed', 'identified_parameters');
    jsonPath = fullfile(dataDir, 'identified_parameters.json');
    matPath  = fullfile(dataDir, 'identified_parameters.mat');

    needBuild = ~isfile(matPath);
    if ~needBuild && isfile(jsonPath)
        j = dir(jsonPath); mt = dir(matPath);
        needBuild = j.datenum > mt.datenum; % JSON edited after last build
    end

    if needBuild
        idp = buildIdentifiedParams(jsonPath, matPath);
        return;
    end

    S = load(matPath, 'idp');
    idp = S.idp;
end
