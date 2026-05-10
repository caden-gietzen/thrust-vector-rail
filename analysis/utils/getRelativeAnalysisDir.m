function relativeAnalysisDir = getRelativeAnalysisDir(scriptPath)
%GETRELATIVEANALYSISDIR Return script folder relative to analysis/.
%
% Example:
% analysis/system_identification/servo_identification/servo_sweep/plot.m
%
% returns:
% system_identification/servo_identification/servo_sweep

    if nargin < 1 || strlength(string(scriptPath)) == 0
        scriptPath = mfilename("fullpath");
    end

    scriptDir = fileparts(scriptPath);
    repoRoot = findRepoRoot(scriptDir);

    analysisRoot = fullfile(repoRoot, "analysis");

    if ~startsWith(scriptDir, analysisRoot)
        error("Script must be inside the analysis folder. Script dir: %s", scriptDir);
    end

    relativeAnalysisDir = erase(scriptDir, analysisRoot);

    if startsWith(relativeAnalysisDir, filesep)
        relativeAnalysisDir = extractAfter(relativeAnalysisDir, 1);
    end
end