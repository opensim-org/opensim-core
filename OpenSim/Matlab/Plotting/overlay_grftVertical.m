function [] = overlay_grftVertical(q, qPlotLabel, time, limb, fileNum)
% Purpose:  Extracts the data column specified by qPlotLabel from q,
%           and plots the data vs time on the current subplot.
%
% Input:    q is a structure with the following format:
%				q.labels 	= array of column labels
%				q.data 		= matrix of data
%				q.nr 		= number of matrix rows
%				q.nc 		= number of matrix columns
%           qPlotLabel specifies the label of the data to be plotted
%           time is an array of corresponding time values
%           limb specifies the limb of interest
%               (needed since plotLabel returns 2 values)
%           fileNum is used to specify the line style
%
% ASA, 11-05, revised 2-06


% Specify attributes of subplots.
pLineStyle = '-';                     
pLineWidth = 1.25;                  
pLineColor = 'k';


% Get data corresponding to plot label.
if isempty(qPlotLabel)
    axis off;  

else
    dataIndex = find(strcmpi(q.labels, qPlotLabel));
    if strcmpi(limb, 'R')
        data = q.data(:, dataIndex(1));
    elseif strcmpi(limb, 'L')
        data = q.data(:, dataIndex(2));
    end

    % Plot data vs time.
    hold on;
    p = plot(time, data);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                'Color', pLineColor);            
end
return;
