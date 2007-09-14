function [] = overlay_muscleDataFromMotSinglePlot(q, qPlotLabel, time, lineColor, lineStyle)
% Purpose:  Extracts the data column(s) specified by qPlotLabel from q,
%           and plots the data vs time on the current subplot.
%
% Input:    q is a structure with the following format:
%				q.labels 	= array of column labels
%				q.data 		= matrix of data
%				q.nr 		= number of matrix rows
%				q.nc 		= number of matrix columns
%           qPlotLabel{} specifies the label(s) of the data to be plotted
%           time is an array of corresponding time values
%
% CTJ, 08-07, based on:
% ASA, 11-05, revised 2-06


% Specify attributes of subplots.
%pLineStyle = {'-', ':', '--', '-.'};
%pLineWidth = 1.25;
% For fast trials: baseColor == [1 0.5 0]
% For free trials: baseColor == [0 1 0.5]
% For slow trials: baseColor == [0 0.5 1]
%pLineColor = baseColor;
%pLineStyle = {'-', '-', '-'};                       
pLineWidth = {2, 2, 2};                  

% Get data corresponding to plot label.
if isempty(qPlotLabel)
    axis off;  

else
    for curveNum = 1:length(qPlotLabel)
        data = q.data(:, strcmpi(q.labels, qPlotLabel{curveNum}));

        % Plot data vs time.
        hold on;
        p = plot(time, data);
        set(p, 'LineStyle', lineStyle, 'LineWidth', ...
                pLineWidth{curveNum}, 'Color', lineColor);
    end    
end
return;
