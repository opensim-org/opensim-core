function [] = plot_GRFTzByLimb(GRFTzData, smCOP, aTime, gapTol, ...
                                                    tInfo, figHandle)
% Purpose:  Generates plots of Fx, Fy, Fz, Tz, COPx, and COPy vs time, 
%           all in the lab coordinate system, for each limb. 
%
% Input:    GRFTzData is a structure with the following format,
%              in the lab coordinate system:
%                          *.Fx(nAnalogFrames)
%                           .Fy(nAnalogFrames)
%                           .Fz(nAnalogFrames)
%                           .Tz(nAnalogFrames)
%                           .COPx(nAnalogFrames)
%                           .COPy(nAnalogFrame)
%                           .startIndex - analog frame number indicating
%                                         start of COP, Tz data
%                           .stopIndex  - analog frame number indicating
%                                         end of COP, Tz data
%           smCOP is a structure with the following format,
%               in the lab coordinate system:
%                       *.COPx(nAnalogFrames), discontinuities removed
%                        .COPy(nAnalogFrames), discontinuities removed
%           aTime is an array of time values corresponding to GRFTzData
%           gapTol is the current value of the tolerance (in analog frames) 
%               used to tweak the index for smoothing/mirroring COP data.
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.trial - trial number to analyze ('character array')
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%           figHandle is the number of the current figure window
%
% ASA, 10-05


% Specify attributes of figure window.
nPlotRows = 4;                      
nPlotCols = 2; 
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
pLineStyle = '-';                  % style for plotting GRFTz data vs time
pLineWidth = 1.5;                  
pLineColor = 'k';
sLineStyle = '-';                  % style for overlaying smoothed COP data
sLineWidth = 1.0;                  
sLineColor = 'r';

% Specify time axis limits.
tmin = 0;
tmax = max(aTime);
tval = tmin:0.5:tmax;

% Generate figure window.
figure(figHandle);    
clf;
set(gcf, 'Position', figPos, 'Color', figColor);

% Plot Fx vs time.
subplot(nPlotRows, nPlotCols, 1);
    p = plot(aTime, GRFTzData.Fx);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    maxFx = max(GRFTzData.Fx);
    minFx = min(GRFTzData.Fx);
    ymax = maxFx + abs(0.1*maxFx);
    ymin = minFx - abs(0.1*minFx);
    set(gca, 'ylim', [ymin ymax], 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    ytickpositions = get(gca, 'Ytick');
    if length(ytickpositions > 5)
        new_ytickpositions = ytickpositions(1:2:length(ytickpositions));
        set(gca, 'Ytick', new_ytickpositions);
    end        
    ylabel('fore-aft GRF (N)');
    a = get(gca, 'ylabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'Box', 'off');
    
% Plot Fy vs time.    
subplot(nPlotRows, nPlotCols, 3);
    p = plot(aTime, GRFTzData.Fy);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    maxFy = max(GRFTzData.Fy);
    minFy = min(GRFTzData.Fy);
    ymax = maxFy + abs(0.1*maxFy);
    ymin = minFy - abs(0.1*minFy);
    set(gca, 'ylim', [ymin ymax], 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    ytickpositions = get(gca, 'Ytick');
    if length(ytickpositions > 5)
        new_ytickpositions = ytickpositions(1:2:length(ytickpositions));
        set(gca, 'Ytick', new_ytickpositions);
    end        
    ylabel('med-lat GRF (N)');
    a = get(gca, 'ylabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'Box', 'off');

% Plot Fz vs time.    
subplot(nPlotRows, nPlotCols, 5);
    p = plot(aTime, GRFTzData.Fz);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    maxFz = max(GRFTzData.Fz);
    ymax = maxFz + 0.1*maxFz;
    ymin = 0;
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
    
% Plot Tz vs time.    
subplot(nPlotRows, nPlotCols, 7);
    p = plot(aTime, GRFTzData.Tz);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    maxTz = max(GRFTzData.Tz);
    minTz = min(GRFTzData.Tz);
    ymax = maxTz + abs(0.1*maxTz);
    ymin = minTz - abs(0.1*minTz);
    set(gca, 'ylim', [ymin ymax], 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    ytickpositions = get(gca, 'Ytick');
    if length(ytickpositions > 5)
        new_ytickpositions = ytickpositions(1:2:length(ytickpositions));
        set(gca, 'Ytick', new_ytickpositions);
    end      
    xlabel('time (s)');
    a = get(gca, 'xlabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'Box', 'off');
    ylabel('vertical torque (Nmm)');
    a = get(gca, 'ylabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'Box', 'off');
        
% Plot COPx vs time.
subplot(nPlotRows, nPlotCols, 2);
    p = plot(aTime, GRFTzData.COPx);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    ytickpositions = get(gca, 'Ytick');
    if length(ytickpositions > 5)
        new_ytickpositions = ytickpositions(1:2:length(ytickpositions));
        set(gca, 'Ytick', new_ytickpositions);
    end        
    ylabel('fore-aft COP (mm)');
    a = get(gca, 'ylabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'Box', 'off');
    
    % Overlay smoothed COP data.
    hold on;
    s = plot(aTime, smCOP.COPx);
    set(s, 'LineStyle', sLineStyle, 'LineWidth', sLineWidth, ...
            'Color', sLineColor);
    
% Plot COPy vs time.
subplot(nPlotRows, nPlotCols, 4);
    p = plot(aTime, GRFTzData.COPy);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, 'FontName', aFontName, ...
        'FontSize', aFontSize, 'TickDir', aTickDir);
    ytickpositions = get(gca, 'Ytick');
    if length(ytickpositions > 5)
        new_ytickpositions = ytickpositions(1:2:length(ytickpositions));
        set(gca, 'Ytick', new_ytickpositions);
    end        
    xlabel('time (s)');
    a = get(gca, 'xlabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);    
    set(gca, 'Box', 'off');    
    ylabel('med-lat COP (mm)');
    a = get(gca, 'ylabel');
    set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'Box', 'off');
    
    % Overlay smoothed COP data.
    hold on;
    s = plot(aTime, smCOP.COPy);
    set(s, 'LineStyle', sLineStyle, 'LineWidth', sLineWidth, ...
            'Color', sLineColor);
    
% Leave subplot6 empty.
subplot(nPlotRows, nPlotCols, 6);
axis off;
        
% Get direction of forward progression, for reference.
if tInfo.FP{1} > tInfo.FP{2}  
    walkDir = '-x';
else
    walkDir = '+x';
end

% Add notes. 
 subplot(nPlotRows, nPlotCols, 8);
    legendString{1} = strcat({'gap tolerance:  '}, num2str(gapTol));
    legendString{2} = strcat({'walk direction:  '}, walkDir);
    for i = 1:length(legendString)
        textPosX = 0.01;
        textPosY = 1 - 0.15*i;
        text(textPosX, textPosY, legendString{i}, 'FontSize', 9);
    end
  axis off;
return;