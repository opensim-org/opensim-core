function [] = plot_ictoFromMkr(fzData, aTime, X, vTime, dX, dt, ...
                eFromGRF, eTimes, threshold, limb, c, tInfo, figHandle)
% Purpose:  Generates plots of vertical GRF, X, and dX vs time,  
%           with IC or TO events overlaid, for the specified limb.
%
% Input:    fzData(aFrameNum, fpHitNum) is a matrix of vertical GRF data
%           aTime is an array of time values corresponding to fzData
%           X is the fore-aft trajectory of the AJC (IC) or TOE (TO)
%           vTime is an array of time values corresponding to X
%           dX is the fore-aft velocity of the AJC (IC) or TOE (TO)
%           dt is an array of time values corresponding to dX
%           eFromGRF(fpHitNum) is an array of IC or TO events,
%               in analog frames, one per FP hit, in the order 
%               specified by tInfo.FP
%           eTimes is an array of times corresponding to IC or TO events
%               for the limb of interest
%           limb specifies the limb for which events are to be plotted
%           c is a structure returned from read_c3DFile()
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.limb  - limbs corresponding to FP strikes (cell array)
%           figHandle is the figure number for plotting events
%
% ASA, 9-05


% Specify attributes of figure window.
nPlotRows = 4;                      
nPlotCols = 1; 
set(0, 'Units', 'pixels');          
scnsize = get(0, 'ScreenSize');
scnwidth = scnsize(3);
scnheight = scnsize(4);
figPos = [0.15*scnwidth, 0.05*scnheight, 0.65*scnwidth, 0.8*scnheight];
figColor = 'w';                
                   
% Specify attributes of subplots.
aFontName = 'helvetica';           % attributes of axis labels and ticks
aFontSize = 8;
aTickDir = 'out';
pLineStyle = '-';                  % style for plotting fz, X, dX vs time
pLineWidth = 1.5;                  
pLineColor = 'k';
eLineStyle = '-';                  % style for overlaying events from GRF 
eLineWidth = 0.75;                   
eLineColor = 'b';
iLineStyle = ':';                  % style for overlaying events from Mkrs 
iLineWidth = 0.75;                   
iLineColor = 'c';
thrLineStyle = '-.';               % style for overlaying threshold
thrLineWidth = 1;
thrLineColor = 'm';

% Specify time axis.
tmin = 0;
tmax = max(aTime);
tval = tmin:1:tmax;

% Generate figure window.
figure(figHandle);    
clf;
set(gcf, 'Position', figPos, 'Color', figColor);

% Plot fz vs time, and overlay events from GRF.
subplot(nPlotRows, nPlotCols, 1);
    maxFz = max(max(fzData));
    limbIndices = strmatch(limb, tInfo.limb);
    for limbHitNum = 1:length(limbIndices)
        fpHitNum = limbIndices(limbHitNum);
        p = plot(aTime, fzData(:, fpHitNum));
        set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                'Color', pLineColor);
        hold on;
    end
    for limbHitNum = 1:length(limbIndices)          % overlay events
        fpHitNum = limbIndices(limbHitNum);
        eventTime = eFromGRF(fpHitNum)/c.analog.rate;
        eT = [eventTime eventTime];
        eY = [0 maxFz];
        e = plot(eT, eY);
        set(e, 'LineStyle', eLineStyle, 'LineWidth', eLineWidth, ...
                'Color', eLineColor);
        hold on;
    end
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    ymin = 0;
    ymax = maxFz + 0.1*maxFz;
    set(gca, 'ylim', [ymin ymax], 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    ytickpositions = get(gca, 'Ytick');
    if length(ytickpositions > 5)
        new_ytickpositions = ytickpositions(1:2:length(ytickpositions));
        set(gca, 'Ytick', new_ytickpositions);
    end        
    ylabel('vertical GRF (N)');
    a = get(gca, 'ylabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'Box', 'off');

% Plot X vs time, and overlay events from GRF and Mkr data.
subplot(nPlotRows, nPlotCols, 2);
    minMkr = min(X);
    maxMkr = max(X);
    p = plot(vTime, X);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    hold on;
    for limbHitNum = 1:length(limbIndices)          % overlay events
        fpHitNum = limbIndices(limbHitNum);
        eventTime = eFromGRF(fpHitNum)/c.analog.rate;
        eT = [eventTime eventTime];
        eY = [minMkr maxMkr];
        e = plot(eT, eY);
        set(e, 'LineStyle', eLineStyle, 'LineWidth', eLineWidth, ...
                'Color', eLineColor);
        hold on;
    end
    for eventNum = 1:length(eTimes)              % overlay events
        eventTime = eTimes(eventNum);
        eT = [eventTime eventTime];
        eY = [minMkr maxMkr];
        i = plot(eT, eY);
        set(i, 'LineStyle', iLineStyle, 'LineWidth', iLineWidth, ...
               'Color', iLineColor);
        hold on;
    end
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    ymin = minMkr - 0.1*(maxMkr - minMkr);
    ymax = maxMkr + 0.1*(maxMkr - minMkr);
    set(gca, 'ylim', [ymin ymax], 'FontName', aFontName, ...
                'FontSize', aFontSize, 'TickDir', aTickDir);
    ytickpositions = get(gca, 'Ytick');
    if length(ytickpositions > 5)
        new_ytickpositions = ytickpositions(1:2:length(ytickpositions));
        set(gca, 'Ytick', new_ytickpositions);
    end        
    ylabel('forward position (mm)');
    a = get(gca, 'ylabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'Box', 'off');   
    
% Plot dX vs time, and overlay events from GRF and Mkr data;
%   overlay threshold for reference.
subplot(nPlotRows, nPlotCols, 3);
    minMkr = min(dX);
    maxMkr = max(dX);
    p = plot(dt, dX);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    hold on;
    for limbHitNum = 1:length(limbIndices)          % overlay events
        fpHitNum = limbIndices(limbHitNum);
        eventTime = eFromGRF(fpHitNum)/c.analog.rate;
        eT = [eventTime eventTime];
        eY = [minMkr maxMkr];
        e = plot(eT, eY);
        set(e, 'LineStyle', eLineStyle, 'LineWidth', eLineWidth, ...
                'Color', eLineColor);
        hold on;
    end
    for eventNum = 1:length(eTimes)              % overlay events
        eventTime = eTimes(eventNum);
        eT = [eventTime eventTime];
        eY = [minMkr maxMkr];
        hold on;
        i = plot(eT, eY);
        set(i, 'LineStyle', iLineStyle, 'LineWidth', iLineWidth, ...
               'Color', iLineColor);
        hold on;
    end
    threshT = [0 dt(length(dt))];                   % overlay threshold
    threshY = [threshold, threshold];
    thr = plot(threshT, threshY);
    set(thr, 'LineStyle', thrLineStyle, 'LineWidth', thrLineWidth, ...
               'Color', thrLineColor);
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    xlabel('time (s)');
    a = get(gca, 'xlabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    ymin = minMkr - 0.1*(maxMkr - minMkr);
    ymax = maxMkr + 0.1*(maxMkr - minMkr);
    set(gca, 'ylim', [ymin ymax], 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    ytickpositions = get(gca, 'Ytick');
    if length(ytickpositions > 5)
        new_ytickpositions = ytickpositions(1:2:length(ytickpositions));
        set(gca, 'Ytick', new_ytickpositions);
    end        
    ylabel('forward velocity (mm/s)');
    a = get(gca, 'ylabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'Box', 'off');   
    
% Display value of threshold.
 subplot(nPlotRows, nPlotCols, 4);
    legendString{1} = strcat({'threshold (mm/s):  '}, ...
        num2str(round(threshold)));
    for i = 1:length(legendString)
        textPosX = 0.01;
        textPosY = 1 - 0.15*i;
        text(textPosX, textPosY, legendString{i}, 'FontSize', 9);
    end
  axis off;
    
return;