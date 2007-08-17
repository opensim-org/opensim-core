function [] = directlyOverlay_perryEMG(perryScaled, refNum, timeRange, fgColor)
% Purpose:  Overlays EMG on/off times reported by Perry, for the muscles in 
%           perryScaled, onto the current subplot.
%
% Input:    perryScaled is a structure with the following format, 
%             with on/off events in units of time, 
%             where t=0 at IC of the 1st FP hit: 
%             *(nRefMusles).muscle    - name of muscle
%                          .onoff     - nx2 matrix of [on off] events, 
%                                       where each row is one 'burst'
%                          .onoffAlt  - 'alternate' on/off timings reported
%           timeRange.min and timeRange.max are the min and max values 
%               of the time axis, respectively
%           fgColor is the color of the Perry bars in the plots
%
% ASA, 9-05, rev 11-05


% Specify attributes of subplots.
pLineStyle = '-';                   % style for Perry's on/off data
pLineWidth = 5.0;                  
pLineColor = fgColor;
qLineStyle = '-';                   % style for Perry's on/off alt data
qLineWidth = 2.0;                  
qLineColor = fgColor;

% Overlay EMG on/off timing from Perry for reference.
% NOTE:  [ymin, ymax] = get(gca, 'ylim'); 
yLim = get(gca, 'ylim');
ymin = yLim(1);
ymax = yLim(2);

dy = (ymax - ymin)/6;
yPos = ymax - dy*refNum;
yArray = [yPos yPos];
clear nOn nCol;
[nOn, nCol] = size(perryScaled.onoff);
for burstNum = 1:nOn
    xArray = [perryScaled.onoff(burstNum, 1) ...
                perryScaled.onoff(burstNum, 2)];
    hold on;                
    p = plot(xArray, yArray);        
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                     'Color', pLineColor);
end
if ~isempty(perryScaled.onoffAlt)
    clear nOn nCol;
    [nOn, nCol] = size(perryScaled.onoffAlt);
    for burstNum = 1:nOn
        xArray = [perryScaled.onoffAlt(burstNum, 1) ...
                    perryScaled.onoffAlt(burstNum, 2)];
        hold on;
        q = plot(xArray, yArray);        
        set(q, 'LineStyle', qLineStyle, 'LineWidth', qLineWidth, ...
                         'Color', qLineColor);
    end
end
text(timeRange.min+0.005, yPos+0.08*ymax, ...
            perryScaled.muscle, 'FontSize', 6);

return;