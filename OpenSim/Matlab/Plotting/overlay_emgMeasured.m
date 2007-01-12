function [] = overlay_emgMeasured(qMeasured, qMeasuredLabel, time)
% Purpose:  Extracts the data column specified by qMeasuredLabel from 
%           qMeasured, and plots the data vs time on the current subplot.
%
% Input:    qMeasured is a structure with the following format:
%				qMeasured.labels 	= array of column labels
%				qMeasured.data 		= matrix of data
%				qMeasured.nr 		= number of matrix rows
%				qMeasured.nc 		= number of matrix columns
%           qMeasuredLabel specifies the label(s) of the data to be plotted
%           time is an array of corresponding time values
%
% ASA, 11-05, revised 2-06


% Specify attributes of subplots.
pLineStyle = {'-', '-', ':', ':'};                       
pLineWidth = {0.25, 0.5, 0.75, 0.75};                  
pLineColor = {[0.65 0.65 0.65], 'g', 'g', 'g'};

% Get data corresponding to plot label.
if ~isempty(qMeasuredLabel)
    for curveNum = 1:length(qMeasuredLabel)
        dataIndex = find(strcmpi(qMeasured.labels, qMeasuredLabel{curveNum}));
        data = qMeasured.data(:, dataIndex);

        % Plot data vs time.
        hold on;
        p = plot(time, data);
        set(p, 'LineStyle', pLineStyle{curveNum}, 'LineWidth', ...
                 pLineWidth{curveNum}, 'Color', pLineColor{curveNum});
    end
end
return;
