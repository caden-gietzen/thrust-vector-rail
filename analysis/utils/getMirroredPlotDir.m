function plotDir = getMirroredPlotDir(scriptPath)
%GETMIRROREDPLOTDIR Get mirrored plots directory for an analysis script.
%
% Example:
% analysis/system_identification/servo_identification/servo_sweep/plot.m
%
% maps to:
% plots/system_identification/servo_identification/servo_sweep

    if nargin < 1 || strlength(string(scriptPath)) == 0
        scriptPath = mfilename("fullpath");
    end

    scriptDir = fileparts(scriptPath);
    repoRoot = findRepoRoot(scriptDir);

    relativeAnalysisDir = getRelativeAnalysisDir(scriptPath);

    plotDir = fullfile(repoRoot, "plots", relativeAnalysisDir);

    if ~isfolder(plotDir)
        mkdir(plotDir);
    end
end