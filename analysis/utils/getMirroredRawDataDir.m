function dataDir = getMirroredRawDataDir(scriptPath)
%GETMIRROREDRAWDATADIR Get mirrored data/raw directory for an analysis script.
%
% Example:
% analysis/system_identification/servo_identification/servo_sweep/plot.m
%
% maps to:
% data/raw/system_identification/servo_identification/servo_sweep

    if nargin < 1 || strlength(string(scriptPath)) == 0
        scriptPath = mfilename("fullpath");
    end

    scriptDir = fileparts(scriptPath);
    repoRoot = findRepoRoot(scriptDir);

    relativeAnalysisDir = getRelativeAnalysisDir(scriptPath);

    dataDir = fullfile(repoRoot, "data", "raw", relativeAnalysisDir);
end