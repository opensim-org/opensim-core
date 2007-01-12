function [] = overlay_emgPerryScaled(perryScaled, figHandle)
% Purpose:  Overlays EMG on/off times reported by Perry, for the muscles in 
%           perryScaled (as specified in get_muscRefList() for the channel 
%           of interest), onto a subplot of the figure window specified by
%           figHandle. 
%
% Input:    perryScaled is a structure with the following format, 
%             with on/off events in units of time, 
%             where t=0 at IC of the 1st FP hit: 
%             *(nRefMusles).muscle    - name of muscle
%                          .onoff     - nx2 matrix of [on off] events, 
%                                       where each row is one 'burst'
%                          .onoffAlt  - 'alternate' on/off timings reported
%           figHandle is the number of the current figure window
%          
% ASA, 9-05


% Specify attributes of figure window.
nPlotRows = 4;                      
nPlotCols = 1;

% Specify attributes of subplots.
tFontName = 'helvetica';            % style for plot title
tFontSize = 9;
tVerticalAlignment = 'middle';
pLineStyle = '-';                   % style for Perry's on/off data
pLineWidth = 6.0;                  
pLineColor = 'k';
qLineStyle = '-';                   % style for Perry's on/off alt data
qLineWidth = 2.5;                  
qLineColor = 'k';

% Overlay EMG on/off timing from Perry for reference.
% NOTE:  [ymin, ymax] = get(gca, 'ylim'); 
figure(figHandle);  
subplot(nPlotRows, nPlotCols, 4);
nRefMuscles = length(perryScaled);
yLim = get(gca, 'ylim');
ymin = yLim(1);
ymax = yLim(2);

dy = (ymax - ymin)/(nRefMuscles + 1);
for refNum = 1:nRefMuscles
    yPos = ymax - dy*refNum;
    yArray = [yPos yPos];
    [nOn, nCol] = size(perryScaled(refNum).onoff);
    for burstNum = 1:nOn
        xArray = [perryScaled(refNum).onoff(burstNum, 1) ...
                    perryScaled(refNum).onoff(burstNum, 2)];
        p = plot(xArray, yArray);        
        set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                         'Color', pLineColor);
        hold on;
    end
    if ~isempty(perryScaled(refNum).onoffAlt)
        [nOn, nCol] = size(perryScaled(refNum).onoffAlt);
        for burstNum = 1:nOn
            xArray = [perryScaled(refNum).onoffAlt(burstNum, 1) ...
                        perryScaled(refNum).onoffAlt(burstNum, 2)];
            q = plot(xArray, yArray);        
            set(q, 'LineStyle', qLineStyle, 'LineWidth', qLineWidth, ...
                             'Color', qLineColor);
            hold on;
        end
    end
    text(0.01, yPos+0.08*ymax, perryScaled(refNum).muscle, 'FontSize', 8);
end
title('EMG On/Off Times from Perry')
    t = get(gca, 'title');
    set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
        'VerticalAlignment', tVerticalAlignment);
return;