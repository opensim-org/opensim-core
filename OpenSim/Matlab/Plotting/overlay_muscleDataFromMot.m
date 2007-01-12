function [] = overlay_muscleDataFromMot(q, qPlotLabel, time, fileNum)
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
%           fileNum is used to specify the line style
%
% ASA, 11-05, revised 2-06


% Specify attributes of subplots.
pLineStyle = {'-', ':', '--', '-.', '-'};                       
pLineWidth = 1.25;                  
pLineColor = {'b', 'r', 'c', 'm', 'g'};

% Get data corresponding to plot label.
if isempty(qPlotLabel)
    axis off;  

else
    for curveNum = 1:length(qPlotLabel)
        dataIndex = find(strcmpi(q.labels, qPlotLabel{curveNum}));
        data = q.data(:, dataIndex);

        % Plot data vs time.
        hold on;
        p = plot(time, data);
        set(p, 'LineStyle', pLineStyle{curveNum}, 'LineWidth', ...
                pLineWidth, 'Color', pLineColor{fileNum});
    end    
end
return;
