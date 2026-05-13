function dataDir = getMirroredRawDataDir(scriptPath, datasetStatus)
%GETMIRROREDRAWDATADIR Get mirrored data/raw directory for an analysis script.
%
% Example:
% analysis/system_identification/servo_identification/servo_sweep/plot.m
%
% maps to:
% data/raw/system_identification/servo_identification/servo_sweep/accepted
%
% datasetStatus options:
%   "accepted"   default; clean data used for analysis
%   "candidate"  newly generated data pending review
%   "rejected"   bad data kept for diagnostics
%   "diagnostics" special troubleshooting data

    if nargin < 1 || strlength(string(scriptPath)) == 0
        scriptPath = mfilename("fullpath");
    end

    if nargin < 2 || strlength(string(datasetStatus)) == 0
        datasetStatus = "accepted";
    end

    validStatuses = ["accepted", "candidate", "rejected", "diagnostics"];
    datasetStatus = string(datasetStatus);

    if ~ismember(datasetStatus, validStatuses)
        error("Invalid datasetStatus '%s'. Must be one of: %s", ...
            datasetStatus, strjoin(validStatuses, ", "));
    end

    scriptDir = fileparts(scriptPath);
    repoRoot = findRepoRoot(scriptDir);

    relativeAnalysisDir = getRelativeAnalysisDir(scriptPath);

    dataDir = fullfile(repoRoot, "data", "raw", relativeAnalysisDir, datasetStatus);
end