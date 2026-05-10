function repoRoot = findRepoRoot(startDir)
%FINDREPOROOT Find the repository root by searching upward for .git.
%
% repoRoot = findRepoRoot()
% repoRoot = findRepoRoot(startDir)

    if nargin < 1 || strlength(string(startDir)) == 0
        startDir = pwd;
    end

    currentDir = char(startDir);

    while true
        gitDir = fullfile(currentDir, ".git");

        if isfolder(gitDir)
            repoRoot = currentDir;
            return;
        end

        parentDir = fileparts(currentDir);

        if strcmp(parentDir, currentDir)
            error("Could not find repo root. No .git folder found above: %s", startDir);
        end

        currentDir = parentDir;
    end
end