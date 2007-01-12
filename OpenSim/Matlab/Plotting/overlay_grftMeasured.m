function [] = overlay_grftMeasured(qMeasured, qMeasuredLabel, time, limb)
% Purpose:  Extracts the data column specified by qMeasuredLabel from 
%           qMeasured, and plots the data vs time on the current subplot.
%
% Input:    qMeasured is a structure with the following format:
%				qMeasured.labels 	= array of column labels
%				qMeasured.data 		= matrix of data
%				qMeasured.nr 		= number of matrix rows
%				qMeasured.nc 		= number of matrix columns
%           qMeasuredLabel specifies the label of the data to be plotted
%           time is an array of corresponding time values
%           limb specifies the limb of interest
%               (needed since plotLabel returns 2 values)
%
% ASA, 11-05, revised 2-06


% Specify attributes of subplots.
pLineStyle = '-';                       
pLineWidth = 1.25;                  
pLineColor = 'k';

% Get data corresponding to qMeasuredLabel.
if isempty(qMeasuredLabel)
    axis off;  

else
    dataIndex = find(strcmpi(qMeasured.labels, qMeasuredLabel));
    if strcmpi(limb, 'R')
        data = qMeasured.data(:, dataIndex(1));
    elseif strcmpi(limb, 'L')
        data = qMeasured.data(:, dataIndex(2));
    end

    % Plot data vs time.
    hold on;
    p = plot(time, data);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                'Color', pLineColor);
end
return;