function [] = overlay_jntmomentsMeasured(qMeasured, qMeasuredLabel, time)
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
    data = qMeasured.data(:, dataIndex);
    
    Nm_perKg = data/1000.0;             % convert Nmm/kg to Nm/kg                                    
        
    % Plot data vs time.
    hold on;
    p = plot(time, Nm_perKg);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                'Color', pLineColor);
end
return;