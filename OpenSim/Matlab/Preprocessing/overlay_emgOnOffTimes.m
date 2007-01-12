function [] = overlay_emgOnOffTimes(onTimes, offTimes, aTime, ...
                        threshold, onWindow, offWindow, figHandle)
% Purpose:  Overlays EMG on/off times, for the EMG channel of interest,
%           onto a subplot of the figure window specified by figHandle. 
%           Current values of the threshold, onWindow, and offWindow 
%           parameters are noted on the figure. 
%
% Input:    onTimes and offTimes are arrays of on/off times, respectively,
%               for the current EMG channel, to the nearest analog frame
%           aTime is an array of times corresponding to the analog data
%           threshold is the value of the normalized EMG envelope at which
%               an on/off event is detected
%           onWindow specifies the number of analog frames > threshold
%               needed for a muscle to be assumed 'on'
%           offWindow specifies the number of analog frames < threshold
%               needed for a muscle to be assumed 'off'
%           figHandle is the number of the current figure window
%          
% ASA, 9-05


% Specify attributes of figure window.
nPlotRows = 4;                      
nPlotCols = 1;

% Specify attributes of subplots.
nLineStyle = '-';                 % line style for overlaying ON events
nLineWidth = 1;
nLineColor = 'b';
fLineStyle = ':';                 % line style for overlaying OFF events
fLineWidth = 1;
fLineColor = 'r';
thrLineStyle = '-';               % line style for overlaying threshold
thrLineWidth = 1.0;
thrLineColor = 'c';

% Overlay on times, off times, and threshold.
figure(figHandle);  
subplot(nPlotRows, nPlotCols, 2);
for onNum = 1:length(onTimes)
    eTime = onTimes(onNum);
    eX = [eTime eTime];
    eY = [0 0.9];
    hold on;
    n = plot(eX, eY);
    set(n, 'LineStyle', nLineStyle, 'LineWidth', nLineWidth, ...
               'Color', nLineColor);
end
for offNum = 1:length(offTimes)
    eTime = offTimes(offNum);
    eX = [eTime eTime];
    eY = [0 0.9];
    hold on;
    f = plot(eX, eY);
    set(f, 'LineStyle', fLineStyle, 'LineWidth', fLineWidth, ...
               'Color', fLineColor);
end
threshX = [0 aTime(length(aTime))];          
threshY = [threshold, threshold];
hold on;
thr = plot(threshX, threshY);
set(thr, 'LineStyle', thrLineStyle, 'LineWidth', thrLineWidth, ...
               'Color', thrLineColor);

% % Display current values of threshold, onWindow, and offWindow.
subplot(nPlotRows, nPlotCols, 3);
legendString{1} = strcat({'EMG CHANNEL '}, num2str(figHandle));
legendString{2} = strcat({'Threshold:  '}, num2str(threshold));
legendString{3} = strcat({'On Window:  '}, num2str(onWindow));
legendString{4} = strcat({'Off Window:  '}, num2str(offWindow));
for i = 1:length(legendString)
    x = 0.01;
    y = 1.15 - 0.15*i;
    text(x, y, legendString{i}, 'FontSize', 8);
end
axis off;

return;
