function [] = plot_ffhoFromMkr(fz, aTime, Z, vTime, dZ, dt, ...
                                    eTimes, threshold, limb, figHandle)
% Purpose:  Generates plots of vertical GRF, Z and dZ vs time,  
%           with FF or HO events overlaid, for the specified limb.
%
% Input:    fz(aFrameNum, cycleNum) is a matrix of vertical GRF data
%               corresponding to the limb of interest
%           aTime is an array of time values corresponding to fz
%           Z is the vertical trajectory of the TOE (FF) or the AJC (HO)
%           vTime is an array of time values corresponding to Z
%           dZ is the vertical velocity of the TOE (FF) or the AJC (HO)
%           dt is an array of time values corresponding to dZ
%           eTimes is an array of times corresponding to FF or HO events
%               for the limb of interest
%           threshold is the threshold value used to detect events
%           limb specifies the limb for which events are to be plotted
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
pLineStyle = '-';                  % style for plotting fz, Z, dZ vs time
pLineWidth = 1.5;                  
pLineColor = 'k';
iLineStyle = ':';                  % style for overlaying events from Mkrs 
iLineWidth = 0.75;                   
iLineColor = 'b';
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

% Plot fz vs time, and overlay events from Mkr data.
subplot(nPlotRows, nPlotCols, 1);
    maxFz = max(max(fz));
    [nAnalogFrames, nCycles] = size(fz);
    for cycleNum = 1:nCycles
        p = plot(aTime, fz(:, cycleNum));
        set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                'Color', pLineColor);
        hold on;
    end
    for eventNum = 1:length(eTimes)              % overlay events
        eventTime = eTimes(eventNum);
        eT = [eventTime eventTime];
        eY = [0 maxFz];
        i = plot(eT, eY);
        set(i, 'LineStyle', iLineStyle, 'LineWidth', iLineWidth, ...
               'Color', iLineColor);
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

% Plot Z vs time, and overlay events from Mkr data;
%   overlay threshold for reference.
subplot(nPlotRows, nPlotCols, 2);
    minMkr = min(Z);
    maxMkr = max(Z);
    p = plot(vTime, Z);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    hold on;
    for eventNum = 1:length(eTimes)              % overlay events
        eventTime = eTimes(eventNum);
        eT = [eventTime eventTime];
        eY = [minMkr maxMkr];
        i = plot(eT, eY);
        set(i, 'LineStyle', iLineStyle, 'LineWidth', iLineWidth, ...
               'Color', iLineColor);
        hold on;
    end
    threshT = [0 vTime(length(vTime))];          % overlay threshold
    threshY = [threshold, threshold];
    thr = plot(threshT, threshY);
    set(thr, 'LineStyle', thrLineStyle, 'LineWidth', thrLineWidth, ...
               'Color', thrLineColor);
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
    ylabel('vertical position (mm)');
    a = get(gca, 'ylabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'Box', 'off');   
    
% Plot dZ vs time, and overlay events from Mkr data.
%   overlay threshold for reference.
subplot(nPlotRows, nPlotCols, 3);
    minMkr = min(dZ);
    maxMkr = max(dZ);
    p = plot(dt, dZ);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    hold on;
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
    ylabel('vertical velocity (mm/s)');
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