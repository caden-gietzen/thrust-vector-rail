function dataFile = selectLatestIndexedCsv(csvFiles)
%SELECTLATESTINDEXEDCSV Select latest CSV by date prefix and numeric suffix.
%
% Expected filename format:
%   yyyy_mm_dd_name_00.csv
%   yyyy_mm_dd_name_01.csv
%   yyyy_mm_dd_name_02.csv
%
% Selection rule:
%   1. Highest date prefix yyyy_mm_dd
%   2. Highest numeric suffix at end of filename
%
% Falls back to modified timestamp if no files match the expected pattern.

    fileNames = string({csvFiles.name});

    bestFile = "";
    bestDate = datetime(0, 1, 1);
    bestIndex = -1;

    pattern = "^(\d{4}_\d{2}_\d{2})_(.+)_(\d+)\.csv$";

    for k = 1:numel(fileNames)
        fileName = fileNames(k);
        tokens = regexp(fileName, pattern, "tokens", "once");

        if isempty(tokens)
            continue;
        end

        fileDate = datetime(tokens{1}, "InputFormat", "yyyy_MM_dd");
        fileIndex = str2double(tokens{3});

        isNewerDate = fileDate > bestDate;
        isSameDateHigherIndex = fileDate == bestDate && fileIndex > bestIndex;

        if isNewerDate || isSameDateHigherIndex
            bestDate = fileDate;
            bestIndex = fileIndex;
            bestFile = fileName;
        end
    end

    if bestFile ~= ""
        dataFile = bestFile;
        return;
    end

    % Fallback: if filenames do not match expected pattern, use timestamp.
    [~, newestIdx] = max([csvFiles.datenum]);
    dataFile = string(csvFiles(newestIdx).name);
end