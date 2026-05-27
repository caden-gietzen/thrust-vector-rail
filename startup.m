thisDir = fileparts(mfilename('fullpath'));
utilsDir = fullfile(thisDir, 'analysis', 'utils');
if isfolder(utilsDir) && ~any(strcmp(strsplit(path, pathsep), utilsDir))
    addpath(utilsDir);
    fprintf('startup: Added analysis/utils to path.\n');
end
