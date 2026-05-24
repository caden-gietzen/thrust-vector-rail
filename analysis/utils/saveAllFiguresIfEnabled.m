function saveAllFiguresIfEnabled(saveFigures, plotDir)
% saveAllFiguresIfEnabled Save all open MATLAB figures to plotDir.
%
% Filenames are based on each figure title when available.
% If plotDir is omitted, saves to a 'figures' subfolder of the current directory.

    if ~saveFigures
        return;
    end

    if nargin < 2 || isempty(plotDir)
        figuresDir = fullfile(pwd, "figures");
    else
        figuresDir = plotDir;
    end

    if ~exist(figuresDir, "dir")
        mkdir(figuresDir);
    end

    figHandles = findall(0, "Type", "figure");

    for i = 1:numel(figHandles)
        fig = figHandles(i);
        figure(fig);

        ax = findall(fig, "Type", "axes");

        figName = "";

        if ~isempty(ax)
            titleText = ax(1).Title.String;

            if iscell(titleText)
                titleText = strjoin(titleText, "_");
            end

            figName = string(titleText);
        end

        if strlength(strtrim(figName)) == 0
            figName = "figure_" + string(fig.Number);
        end

        figName = lower(strtrim(figName));
        figName = regexprep(figName, "\s+", "_");
        figName = regexprep(figName, "[^\w\-]", "");

        pngPath = fullfile(figuresDir, figName + ".png");
        figPath = fullfile(figuresDir, figName + ".fig");

        exportgraphics(fig, pngPath, "Resolution", 300);
        savefig(fig, figPath);

        fprintf("Saved figure:\n  %s\n", pngPath);
    end
end