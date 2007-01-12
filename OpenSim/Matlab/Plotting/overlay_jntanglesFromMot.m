function [] = overlay_jntanglesFromMot(q, qPlotLabel, time, fileNum)
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
%           fileNum is used to specify the line style
%
% ASA, 11-05, revised 2-06


% Specify attributes of subplots.
pLineStyle = {':', '--', '-.', '-', ':'};                       
pLineWidth = 1.25;                  
pLineColor = {'c', 'm', 'b', 'r', 'g'};

zeroLine = zeros(length(time), 1);
zLineStyle = ':';
zLineWidth = 0.25;
zLineColor = [0.3 0.3 0.3];

% Get data corresponding to qPlotLabel.
if isempty(qPlotLabel)
    axis off;  

else
    dataIndex = find(strcmpi(q.labels, qPlotLabel));
    data = q.data(:, dataIndex);

    % Convert pelvis, hip, and knee angles from the model CS to a 
    % conventional gait lab CS.
    if strcmpi(qPlotLabel, 'pelvis_list')
        data = -1*data;
    elseif strcmpi(qPlotLabel, 'pelvis_tilt')
        data = -1*(data - 12);
    elseif strcmpi(qPlotLabel, 'hip_flexion_r')
        data = data + 12;
    elseif strcmpi(qPlotLabel, 'knee_angle_r')
        data = -1*data;
    elseif strcmpi(qPlotLabel, 'hip_flexion_l')
        data = data + 12;
    elseif strcmpi(qPlotLabel, 'knee_angle_l')
        data = -1*data;
    end

    % Plot data vs time.
    hold on;
    p = plot(time, data);
    set(p, 'LineStyle', pLineStyle{fileNum}, 'LineWidth', pLineWidth, ...
                'Color', pLineColor{fileNum});
    
    % Overlay zero line on all plots except pelvis translations.
    pelvisTranslationFlag = strmatch('pelvis_t', qPlotLabel);
    pelvisTiltFlag = strmatch('pelvis_tilt', qPlotLabel);
    if isempty(pelvisTranslationFlag) | ~isempty(pelvisTiltFlag)
        hold on;
        z = plot(time, zeroLine);
        set(z, 'LineWidth', zLineWidth, 'Color', zLineColor);
    end
end
return;