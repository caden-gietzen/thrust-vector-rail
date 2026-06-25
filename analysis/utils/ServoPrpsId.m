classdef ServoPrpsId
    % ServoPrpsId Shared helpers for servo PRPS frequency-domain ID.

    methods (Static)
        function T = readCombinedCsvTables(csvFileStruct)
            tables = cell(numel(csvFileStruct), 1);
            allVarNames = strings(0, 1);

            for k = 1:numel(csvFileStruct)
                thisPath = fullfile(csvFileStruct(k).folder, csvFileStruct(k).name);
                Tk = readtable(thisPath);

                Tk.source_file = repmat(string(csvFileStruct(k).name), height(Tk), 1);
                Tk.source_file_index = repmat(k, height(Tk), 1);

                tables{k} = Tk;
                allVarNames = union(allVarNames, string(Tk.Properties.VariableNames), "stable");
            end

            for k = 1:numel(tables)
                Tk = tables{k};
                currentVars = string(Tk.Properties.VariableNames);

                for v = 1:numel(allVarNames)
                    varName = allVarNames(v);

                    if ~ismember(varName, currentVars)
                        if any(varName == ["source_file", "run_name", "segment"])
                            Tk.(varName) = repmat("", height(Tk), 1);
                        else
                            Tk.(varName) = NaN(height(Tk), 1);
                        end
                    end
                end

                Tk = Tk(:, cellstr(allVarNames));
                tables{k} = Tk;
            end

            T = vertcat(tables{:});
        end

        function T = normalizeServoPrpsTable(T, countsPerRev, defaultServoCenterUs, defaultFreqLabel, servoOutputSign)
            vars = string(T.Properties.VariableNames);

            if ~ismember("t_s", vars)
                if ismember("t_ms", vars)
                    T.t_ms = ServoPrpsId.forceNumeric(T.t_ms);
                    T.t_s = T.t_ms / 1000;
                else
                    error("CSV must contain either t_s or t_ms.");
                end
            else
                T.t_s = ServoPrpsId.forceNumeric(T.t_s);
            end

            if ~ismember("servo_us", vars)
                error("CSV must contain servo_us.");
            end
            T.servo_us = ServoPrpsId.forceNumeric(T.servo_us);

            if ismember("count_delta", vars)
                T.count_delta = ServoPrpsId.forceNumeric(T.count_delta);
            end

            if ~ismember("theta_rad", vars)
                if ismember("count_delta", string(T.Properties.VariableNames))
                    T.theta_rad = T.count_delta / countsPerRev * 2*pi;
                else
                    error("CSV must contain either theta_rad or count_delta.");
                end
            else
                T.theta_rad = ServoPrpsId.forceNumeric(T.theta_rad);
            end

            T.theta_rad = servoOutputSign * T.theta_rad;

            if ismember("theta_deg", string(T.Properties.VariableNames))
                T.theta_deg = ServoPrpsId.forceNumeric(T.theta_deg);
                T.theta_deg = servoOutputSign * T.theta_deg;
            else
                T.theta_deg = T.theta_rad * 180/pi;
            end

            if ~ismember("segment", vars)
                T.segment = repmat("prps", height(T), 1);
            else
                T.segment = string(T.segment);
            end

            if ~ismember("run_idx", vars)
                T.run_idx = zeros(height(T), 1);
            else
                T.run_idx = ServoPrpsId.forceNumeric(T.run_idx);
            end

            if ~ismember("source_file", string(T.Properties.VariableNames))
                T.source_file = repmat("", height(T), 1);
            else
                T.source_file = string(T.source_file);
            end

            ampUs = ServoPrpsId.inferAmplitudeFromSourceFile(T.source_file);

            if any(isnan(ampUs)) && ismember("run_name", vars)
                ampFromRunName = ServoPrpsId.inferAmplitudeFromRunName(string(T.run_name));
                missingAmp = isnan(ampUs) & isfinite(ampFromRunName);
                ampUs(missingAmp) = ampFromRunName(missingAmp);
            end

            T.run_amplitude_us = ampUs;

            if ~ismember("run_name", vars)
                runName = strings(height(T), 1);

                for i = 1:height(T)
                    if isfinite(ampUs(i))
                        runName(i) = "local_amp" + string(round(ampUs(i))) + "_" + defaultFreqLabel;
                    else
                        runName(i) = "run_idx_" + string(T.run_idx(i));
                    end
                end

                T.run_name = runName;
            else
                T.run_name = string(T.run_name);
                emptyName = strlength(strtrim(T.run_name)) == 0;

                for i = 1:height(T)
                    if emptyName(i) && isfinite(ampUs(i))
                        T.run_name(i) = "local_amp" + string(round(ampUs(i))) + "_" + defaultFreqLabel;
                    end
                end
            end

            if ~ismember("command_delta_us", vars)
                T.command_delta_us = T.servo_us - defaultServoCenterUs;
            else
                T.command_delta_us = ServoPrpsId.forceNumeric(T.command_delta_us);
            end

            if ~ismember("period_index", vars)
                T.period_index = zeros(height(T), 1);
            else
                T.period_index = ServoPrpsId.forceNumeric(T.period_index);
            end

            if ~ismember("period_sample_index", vars)
                if ismember("sample_index", vars)
                    T.period_sample_index = ServoPrpsId.forceNumeric(T.sample_index);
                else
                    T.period_sample_index = (0:height(T)-1).';
                end
            else
                T.period_sample_index = ServoPrpsId.forceNumeric(T.period_sample_index);
            end
        end

        function D = getRunData(T, runName, useOnlyPrpsSegment)
            if isempty(T)
                D = table();
                return;
            end

            if ~ismember("run_name", string(T.Properties.VariableNames))
                error("Table has not been normalized. Missing run_name.");
            end

            idx = strcmp(string(T.run_name), string(runName));

            if useOnlyPrpsSegment && ismember("segment", string(T.Properties.VariableNames))
                idx = idx & strcmpi(string(T.segment), "prps");
            end

            D = T(idx, :);
            D = ServoPrpsId.cleanServoPrpsTable(D);
        end

        function D = getSourceFileData(T, sourceFileName, useOnlyPrpsSegment)
            if isempty(T)
                D = table();
                return;
            end

            if ~ismember("source_file", string(T.Properties.VariableNames))
                error("Table has no source_file column. Use readCombinedCsvTables first.");
            end

            idx = strcmp(string(T.source_file), string(sourceFileName));

            if useOnlyPrpsSegment && ismember("segment", string(T.Properties.VariableNames))
                idx = idx & strcmpi(string(T.segment), "prps");
            end

            D = T(idx, :);
            D = ServoPrpsId.cleanServoPrpsTable(D);
        end

        function D = cleanServoPrpsTable(D)
            if isempty(D)
                return;
            end

            D.t_s = ServoPrpsId.forceNumeric(D.t_s);
            D.servo_us = ServoPrpsId.forceNumeric(D.servo_us);
            D.command_delta_us = ServoPrpsId.forceNumeric(D.command_delta_us);
            D.theta_rad = ServoPrpsId.forceNumeric(D.theta_rad);
            D.period_index = ServoPrpsId.forceNumeric(D.period_index);
            D.period_sample_index = ServoPrpsId.forceNumeric(D.period_sample_index);

            valid = isfinite(D.t_s) & ...
                    isfinite(D.command_delta_us) & ...
                    isfinite(D.theta_rad) & ...
                    isfinite(D.period_index) & ...
                    isfinite(D.period_sample_index);

            D = D(valid, :);

            if isempty(D)
                return;
            end

            D = ServoPrpsId.zeroServoAnglePerSourceFile(D);
            D.t_s = D.t_s - D.t_s(1);
        end

        function D = zeroServoAnglePerSourceFile(D)
            if isempty(D)
                return;
            end

            if ismember("source_file_index", string(D.Properties.VariableNames))
                groups = ServoPrpsId.forceNumeric(D.source_file_index);
            else
                groups = ones(height(D), 1);
            end

            uniqueGroups = unique(groups, "stable");

            for k = 1:numel(uniqueGroups)
                idx = groups == uniqueGroups(k);

                if ~any(idx)
                    continue;
                end

                firstIdx = find(idx, 1, "first");
                theta0 = D.theta_rad(firstIdx);
                D.theta_rad(idx) = D.theta_rad(idx) - theta0;

                if ismember("count_delta", string(D.Properties.VariableNames))
                    count0 = D.count_delta(firstIdx);
                    D.count_delta(idx) = D.count_delta(idx) - count0;
                end
            end
        end

        function frf = estimatePrpsFrfFromTable(D, runName, datasetLabel, opts)
            frf = ServoPrpsId.emptyFrf();

            if height(D) < opts.minSamplesPerPeriod
                return;
            end

            if ismember("source_file_index", string(D.Properties.VariableNames))
                fileGroups = double(D.source_file_index);
            else
                fileGroups = ones(height(D), 1);
            end

            uniqueFiles = unique(fileGroups, "stable");

            allU = [];
            allY = [];
            allF = [];
            allBins = [];
            allFileIndex = [];
            allPeriodIndex = [];
            inferredFreqsHz = [];

            for fg = 1:numel(uniqueFiles)
                fileIdx = uniqueFiles(fg);
                Df = D(fileGroups == fileIdx, :);

                periodIds = unique(Df.period_index, "stable");
                periodIds = periodIds(periodIds >= 0);

                if numel(periodIds) < opts.minPeriodsPerRun
                    continue;
                end

                for p = 1:numel(periodIds)
                    periodId = periodIds(p);
                    Dp = Df(Df.period_index == periodId, :);

                    [uPeriod, yPeriod, tPeriod] = ServoPrpsId.makeUniformPeriodVectors(Dp);

                    if numel(uPeriod) < opts.minSamplesPerPeriod
                        continue;
                    end

                    if opts.removePeriodMean
                        uPeriod = uPeriod - mean(uPeriod, "omitnan");
                        yPeriod = yPeriod - mean(yPeriod, "omitnan");
                    end

                    if opts.detrendEachPeriod
                        uPeriod = detrend(uPeriod);
                        yPeriod = detrend(yPeriod);
                    end

                    if opts.useNominalCommandDtForFrf
                        dt_s = opts.nominalCommandDtS;
                    else
                        dt_s = median(diff(tPeriod), "omitnan");
                    end

                    if ~isfinite(dt_s) || dt_s <= 0
                        continue;
                    end

                    N = numel(uPeriod);
                    period_s = N * dt_s;

                    Ufft = fft(uPeriod) / N;
                    Yfft = fft(yPeriod) / N;

                    positiveBins = (1:floor(N/2)).';
                    fHz = positiveBins / period_s;
                    freqMask = fHz >= opts.freqMinHz & fHz <= opts.freqMaxHz;

                    if opts.inferFreqsFromInput && isempty(inferredFreqsHz)
                        Uabs = abs(Ufft(positiveBins));
                        UabsMasked = Uabs;
                        UabsMasked(~freqMask) = 0;

                        maxU = max(UabsMasked, [], "omitnan");

                        if maxU <= 0 || ~isfinite(maxU)
                            continue;
                        end

                        keep = UabsMasked >= maxU * opts.inputBinRelativeThreshold & freqMask;
                        inferredFreqsHz = fHz(keep);
                        inferredFreqsHz = unique(round(inferredFreqsHz, 10), "stable");
                    end

                    if opts.inferFreqsFromInput
                        targetFreqsHz = inferredFreqsHz;
                    else
                        targetFreqsHz = opts.freqListHz(:);
                    end

                    if isempty(targetFreqsHz)
                        continue;
                    end

                    for q = 1:numel(targetFreqsHz)
                        [~, localIdx] = min(abs(fHz - targetFreqsHz(q)));
                        bin = positiveBins(localIdx);
                        fActual = fHz(localIdx);

                        if fActual < opts.freqMinHz || fActual > opts.freqMaxHz
                            continue;
                        end

                        allU(end+1, 1) = Ufft(bin); %#ok<AGROW>
                        allY(end+1, 1) = Yfft(bin); %#ok<AGROW>
                        allF(end+1, 1) = fActual; %#ok<AGROW>
                        allBins(end+1, 1) = bin; %#ok<AGROW>
                        allFileIndex(end+1, 1) = fileIdx; %#ok<AGROW>
                        allPeriodIndex(end+1, 1) = periodId; %#ok<AGROW>
                    end
                end
            end

            if isempty(allF)
                return;
            end

            uniqueFreqs = unique(round(allF, 10), "stable");
            G_emp = NaN(numel(uniqueFreqs), 1);
            coherence = NaN(numel(uniqueFreqs), 1);
            Suu = NaN(numel(uniqueFreqs), 1);
            Syy = NaN(numel(uniqueFreqs), 1);
            Syu = NaN(numel(uniqueFreqs), 1);
            nAvg = zeros(numel(uniqueFreqs), 1);

            for i = 1:numel(uniqueFreqs)
                idx = abs(allF - uniqueFreqs(i)) < 1e-9;
                U = allU(idx);
                Y = allY(idx);
                valid = isfinite(U) & isfinite(Y) & abs(U) > 0;
                U = U(valid);
                Y = Y(valid);

                if isempty(U)
                    continue;
                end

                Syu_i = mean(Y .* conj(U), "omitnan");
                Suu_i = mean(abs(U).^2, "omitnan");
                Syy_i = mean(abs(Y).^2, "omitnan");

                G_emp(i) = Syu_i / max(Suu_i, eps);
                coherence(i) = abs(Syu_i)^2 / max(Suu_i * Syy_i, eps);
                Suu(i) = Suu_i;
                Syy(i) = Syy_i;
                Syu(i) = Syu_i;
                nAvg(i) = numel(U);
            end

            validFrf = isfinite(G_emp) & isfinite(coherence) & isfinite(uniqueFreqs(:));

            frf.f_Hz = uniqueFreqs(validFrf);
            frf.w_rad_s = 2*pi*frf.f_Hz;
            frf.G_emp = G_emp(validFrf);
            frf.coherence = coherence(validFrf);
            frf.Suu = Suu(validFrf);
            frf.Syy = Syy(validFrf);
            frf.Syu = Syu(validFrf);
            frf.n_avg = nAvg(validFrf);
            frf.run_name = string(runName);
            frf.dataset_label = string(datasetLabel);
            frf.raw_U = allU;
            frf.raw_Y = allY;
            frf.raw_f_Hz = allF;
            frf.raw_bins = allBins;
            frf.raw_file_index = allFileIndex;
            frf.raw_period_index = allPeriodIndex;

            [frf.f_Hz, sortIdx] = sort(frf.f_Hz);
            frf.w_rad_s = frf.w_rad_s(sortIdx);
            frf.G_emp = frf.G_emp(sortIdx);
            frf.coherence = frf.coherence(sortIdx);
            frf.Suu = frf.Suu(sortIdx);
            frf.Syy = frf.Syy(sortIdx);
            frf.Syu = frf.Syu(sortIdx);
            frf.n_avg = frf.n_avg(sortIdx);
        end

        function [uPeriod, yPeriod, tPeriod, sampleIndex] = makeUniformPeriodVectors(Dp)
            sampleIndexRaw = ServoPrpsId.forceNumeric(Dp.period_sample_index);
            uRaw = ServoPrpsId.forceNumeric(Dp.command_delta_us);
            yRaw = ServoPrpsId.forceNumeric(Dp.theta_rad);
            tRaw = ServoPrpsId.forceNumeric(Dp.t_s);

            valid = isfinite(sampleIndexRaw) & isfinite(uRaw) & isfinite(yRaw) & isfinite(tRaw);
            sampleIndexRaw = sampleIndexRaw(valid);
            uRaw = uRaw(valid);
            yRaw = yRaw(valid);
            tRaw = tRaw(valid);

            if isempty(sampleIndexRaw)
                uPeriod = [];
                yPeriod = [];
                tPeriod = [];
                sampleIndex = [];
                return;
            end

            sampleIndex = unique(sampleIndexRaw, "stable");
            sampleIndex = sort(sampleIndex);

            uPeriod = NaN(numel(sampleIndex), 1);
            yPeriod = NaN(numel(sampleIndex), 1);
            tPeriod = NaN(numel(sampleIndex), 1);

            for i = 1:numel(sampleIndex)
                idx = sampleIndexRaw == sampleIndex(i);
                uPeriod(i) = mean(uRaw(idx), "omitnan");
                yPeriod(i) = mean(yRaw(idx), "omitnan");
                tPeriod(i) = mean(tRaw(idx), "omitnan");
            end

            valid = isfinite(uPeriod) & isfinite(yPeriod) & isfinite(tPeriod);
            uPeriod = uPeriod(valid);
            yPeriod = yPeriod(valid);
            tPeriod = tPeriod(valid);
            sampleIndex = sampleIndex(valid);

            [sampleIndex, sortIdx] = sort(sampleIndex);
            uPeriod = uPeriod(sortIdx);
            yPeriod = yPeriod(sortIdx);
            tPeriod = tPeriod(sortIdx);
        end

        function model = fitOneFrequencyModel(modelType, f_Hz, G_emp, coherence, opts)
            w = 2*pi*f_Hz(:);
            G_emp = G_emp(:);
            coherence = coherence(:);

            if strcmpi(opts.gainFloorMode, "fraction_of_median")
                gainFloor = opts.gainFloorFractionOfMedian * median(abs(G_emp), "omitnan");
            elseif strcmpi(opts.gainFloorMode, "manual")
                gainFloor = opts.manualGainFloor;
            else
                error("Unknown gainFloorMode: %s", opts.gainFloorMode);
            end

            if ~isfinite(gainFloor) || gainFloor <= 0
                gainFloor = opts.manualGainFloor;
            end

            initialK = ServoPrpsId.clampScalar(opts.initialK, opts.minGainAbs, opts.maxGainAbs);
            initialTau = ServoPrpsId.clampScalar(opts.initialTau, opts.minTau, opts.maxTau);
            initialTau2 = ServoPrpsId.clampScalar(opts.initialTau2, opts.minTau, opts.maxTau);
            initialDelay = ServoPrpsId.clampScalar(opts.initialDelay, opts.minDelay, opts.maxDelay);

            p0 = ServoPrpsId.packParams(modelType, initialK, initialTau, initialTau2, initialDelay);

            costFun = @(p) ServoPrpsId.frequencyFitCost(p, modelType, w, G_emp, coherence, opts, gainFloor);

            searchOpts = optimset( ...
                "Display", "off", ...
                "MaxIter", opts.maxIter, ...
                "MaxFunEvals", opts.maxEval, ...
                "TolX", 1e-10, ...
                "TolFun", 1e-12);

            [pBest, bestCost] = fminsearch(costFun, p0, searchOpts);
            params = ServoPrpsId.unpackParams(modelType, pBest, opts);
            G_fit = ServoPrpsId.evalFrequencyModel(modelType, params, w);
            err = ServoPrpsId.weightedComplexError(G_fit, G_emp, coherence, opts.weightingMode, gainFloor);
            weightedError = sqrt(mean(abs(err).^2, "omitnan"));

            model.model_type = string(modelType);
            model.params = params;
            model.K = params.K;
            model.tau1_s = params.tau1_s;
            model.tau2_s = params.tau2_s;
            model.delay_s = params.delay_s;
            model.best_cost = bestCost;
            model.fit_weighted_error = weightedError;
            model.gain_floor = gainFloor;
            model.weighting_mode = string(opts.weightingMode);
            model.train_weighted_error = weightedError;

            [magRmseDb, phaseRmseDeg] = ServoPrpsId.bodeErrorMetrics(G_fit, G_emp);
            model.train_mag_rmse_dB = magRmseDb;
            model.train_phase_rmse_deg = phaseRmseDeg;
        end

        function modelOut = scoreSingleModelOnFrf(modelIn, frf)
            modelOut = modelIn;

            if isempty(frf.f_Hz)
                modelOut.validation_weighted_error = NaN;
                modelOut.validation_mag_rmse_dB = NaN;
                modelOut.validation_phase_rmse_deg = NaN;
                return;
            end

            G_fit = ServoPrpsId.evalFrequencyModel(modelIn.model_type, modelIn.params, frf.w_rad_s);

            err = ServoPrpsId.weightedComplexError( ...
                G_fit, ...
                frf.G_emp, ...
                frf.coherence, ...
                modelIn.weighting_mode, ...
                modelIn.gain_floor);

            modelOut.validation_weighted_error = sqrt(mean(abs(err).^2, "omitnan"));

            [magRmseDb, phaseRmseDeg] = ServoPrpsId.bodeErrorMetrics(G_fit, frf.G_emp);
            modelOut.validation_mag_rmse_dB = magRmseDb;
            modelOut.validation_phase_rmse_deg = phaseRmseDeg;
        end

        function G = evalFrequencyModel(modelType, params, w)
            s = 1j*w;

            switch string(modelType)
                case "first_order"
                    G = params.K ./ (params.tau1_s*s + 1);
                case "first_order_delay"
                    G = params.K .* exp(-s*params.delay_s) ./ (params.tau1_s*s + 1);
                case "second_order_lag"
                    G = params.K ./ ((params.tau1_s*s + 1) .* (params.tau2_s*s + 1));
                case "second_order_lag_delay"
                    G = params.K .* exp(-s*params.delay_s) ./ ...
                        ((params.tau1_s*s + 1) .* (params.tau2_s*s + 1));
                otherwise
                    error("Unknown modelType: %s", modelType);
            end
        end

        function [magRmseDb, phaseRmseDeg] = bodeErrorMetrics(G_fit, G_emp)
            magFitDb = 20*log10(abs(G_fit));
            magEmpDb = 20*log10(abs(G_emp));

            phaseFit = unwrap(angle(G_fit));
            phaseEmp = unwrap(angle(G_emp));

            phaseErrDeg = (phaseFit - phaseEmp) * 180/pi;
            magErrDb = magFitDb - magEmpDb;

            magRmseDb = sqrt(mean(magErrDb.^2, "omitnan"));
            phaseRmseDeg = sqrt(mean(phaseErrDeg.^2, "omitnan"));
        end

        function x = forceNumeric(x)
            if isnumeric(x)
                x = double(x);
                return;
            end

            if iscell(x)
                x = string(x);
            end

            if isstring(x) || ischar(x) || iscategorical(x)
                x = str2double(string(x));
                return;
            end

            try
                x = double(x);
            catch
                x = NaN(size(x));
            end
        end

        function ampUs = inferAmplitudeFromSourceFile(sourceFile)
            sourceFile = string(sourceFile);
            ampUs = NaN(numel(sourceFile), 1);

            for i = 1:numel(sourceFile)
                token = regexp(sourceFile(i), "amp(\d+)", "tokens", "once");

                if ~isempty(token)
                    ampUs(i) = str2double(token{1});
                end
            end
        end

        function ampUs = inferAmplitudeFromRunName(runName)
            runName = string(runName);
            ampUs = NaN(numel(runName), 1);

            for i = 1:numel(runName)
                token = regexp(runName(i), "amp(\d+)", "tokens", "once");

                if ~isempty(token)
                    ampUs(i) = str2double(token{1});
                end
            end
        end

        function frf = emptyFrf()
            frf.f_Hz = [];
            frf.w_rad_s = [];
            frf.G_emp = [];
            frf.coherence = [];
            frf.Suu = [];
            frf.Syy = [];
            frf.Syu = [];
            frf.n_avg = [];
            frf.run_name = "";
            frf.dataset_label = "";
            frf.raw_U = [];
            frf.raw_Y = [];
            frf.raw_f_Hz = [];
            frf.raw_bins = [];
            frf.raw_file_index = [];
            frf.raw_period_index = [];
        end
    end

    methods (Static, Access = private)
        function J = frequencyFitCost(p, modelType, w, G_emp, coherence, opts, gainFloor)
            params = ServoPrpsId.unpackParams(modelType, p, opts);
            G_fit = ServoPrpsId.evalFrequencyModel(modelType, params, w);
            err = ServoPrpsId.weightedComplexError(G_fit, G_emp, coherence, opts.weightingMode, gainFloor);
            J = mean(abs(err).^2, "omitnan");

            if ~isfinite(J)
                J = 1e30;
            end
        end

        function err = weightedComplexError(G_fit, G_emp, coherence, weightingMode, gainFloor)
            switch string(weightingMode)
                case "relative_complex"
                    denom = max(abs(G_emp), gainFloor);
                    err = (G_fit - G_emp) ./ denom;
                case "relative_with_coh"
                    denom = max(abs(G_emp), gainFloor);
                    cohWeight = sqrt(max(coherence, 0));
                    err = cohWeight .* (G_fit - G_emp) ./ denom;
                case "absolute_complex"
                    err = G_fit - G_emp;
                otherwise
                    error("Unknown weightingMode: %s", weightingMode);
            end
        end

        function params = unpackParams(modelType, p, opts)
            switch string(modelType)
                case "first_order"
                    K = ServoPrpsId.expClamp(p(1), opts.minGainAbs, opts.maxGainAbs);
                    tau1 = ServoPrpsId.expClamp(p(2), opts.minTau, opts.maxTau);
                    params.K = K;
                    params.tau1_s = tau1;
                    params.tau2_s = NaN;
                    params.delay_s = 0;
                case "first_order_delay"
                    K = ServoPrpsId.expClamp(p(1), opts.minGainAbs, opts.maxGainAbs);
                    tau1 = ServoPrpsId.expClamp(p(2), opts.minTau, opts.maxTau);
                    delay = ServoPrpsId.expClamp(p(3), max(opts.minDelay, 1e-6), opts.maxDelay);
                    params.K = K;
                    params.tau1_s = tau1;
                    params.tau2_s = NaN;
                    params.delay_s = delay;
                case "second_order_lag"
                    K = ServoPrpsId.expClamp(p(1), opts.minGainAbs, opts.maxGainAbs);
                    tau1 = ServoPrpsId.expClamp(p(2), opts.minTau, opts.maxTau);
                    tau2 = ServoPrpsId.expClamp(p(3), opts.minTau, opts.maxTau);
                    params.K = K;
                    params.tau1_s = max(tau1, tau2);
                    params.tau2_s = min(tau1, tau2);
                    params.delay_s = 0;
                case "second_order_lag_delay"
                    K = ServoPrpsId.expClamp(p(1), opts.minGainAbs, opts.maxGainAbs);
                    tau1 = ServoPrpsId.expClamp(p(2), opts.minTau, opts.maxTau);
                    tau2 = ServoPrpsId.expClamp(p(3), opts.minTau, opts.maxTau);
                    delay = ServoPrpsId.expClamp(p(4), max(opts.minDelay, 1e-6), opts.maxDelay);
                    params.K = K;
                    params.tau1_s = max(tau1, tau2);
                    params.tau2_s = min(tau1, tau2);
                    params.delay_s = delay;
                otherwise
                    error("Unknown modelType: %s", modelType);
            end
        end

        function p = packParams(modelType, K, tau1, tau2, delay)
            K = max(K, 1e-12);

            switch string(modelType)
                case "first_order"
                    p = [log(K), log(tau1)];
                case "first_order_delay"
                    p = [log(K), log(tau1), log(max(delay, 1e-6))];
                case "second_order_lag"
                    p = [log(K), log(tau1), log(tau2)];
                case "second_order_lag_delay"
                    p = [log(K), log(tau1), log(tau2), log(max(delay, 1e-6))];
                otherwise
                    error("Unknown modelType: %s", modelType);
            end
        end

        function x = expClamp(p, lo, hi)
            x = exp(p);
            x = ServoPrpsId.clampScalar(x, lo, hi);
        end

        function y = clampScalar(x, lo, hi)
            y = min(max(x, lo), hi);
        end
    end
end
