function [] = overlay_jntanglesMeasured(qMeasured, qMeasuredLabel, time)
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
pLineColor = [0.4, 0.4, 0.4];

% Get data corresponding to qMeasuredLabel.
if isempty(qMeasuredLabel)
    axis off;  

else
    dataIndex = find(strcmpi(qMeasured.labels, qMeasuredLabel));
    data = qMeasured.data(:, dataIndex);

    % Convert pelvis, hip, knee, and trunk angles from the model CS to a 
    % conventional gait lab CS.
    if strcmpi(qMeasuredLabel, 'pelvis_list')
        data = -1*data;
    elseif strcmpi(qMeasuredLabel, 'pelvis_tilt')
        data = -1*(data - 12);
    elseif strcmpi(qMeasuredLabel, 'hip_flexion_r')
        data = data + 12;
    elseif strcmpi(qMeasuredLabel, 'knee_angle_r')
        data = -1*data;
    elseif strcmpi(qMeasuredLabel, 'hip_flexion_l')
        data = data + 12;
    elseif strcmpi(qMeasuredLabel, 'knee_angle_l')
        data = -1*data;
%     elseif strcmpi(qMeasuredLabel, 'lumbar_extension')
%         data = -1*data;     % NOTE: magnitude not converted
    end

    % Plot data vs time.
    hold on;
    p = plot(time, data);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                'Color', pLineColor);
end
return;