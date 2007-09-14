function [] = overlay_jointMomentsFromMotSinglePlot(q, qPlotLabel, time, lineColors)
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
% CTJ, 02-07, adapted from:
% ASA, 11-05, revised 2-06


% Specify attributes of subplots.
pLineStyle = {'-', '-', '-', '-'};                       
pLineWidth = {6, 2, 6, 2};                  
pLineColor = {lineColors{1}, lineColors{2}, [0.7 0 0], [1 0 0]};

% Get data corresponding to plot label.
if isempty(qPlotLabel)
    axis off;  

else
    for curveNum = 1:length(qPlotLabel)
        data = q.data(:, strcmpi(q.labels, qPlotLabel{curveNum}));

        % Convert pelvis, hip, and knee angles from the model CS to a 
        % conventional gait lab CS.
        %if any(strcmpi(qPlotLabel, {'pelvis_list_ik_moment' 'pelvis_list_rra1_moment' 'pelvis_list_rra2_moment' 'pelvis_list_cmc_moment'}))
        %    data = -1*data;
        %elseif any(strcmpi(qPlotLabel, {'pelvis_tilt_ik_moment' 'pelvis_tilt_rra1_moment' 'pelvis_tilt_rra2_moment' 'pelvis_tilt_cmc_moment'}))
            % I'm not sure if this is right (Chand)
        %    data = -1*data;
            %data = -1*(data - 12);
        % I'm not sure if this is right either: (Chand)
        %elseif strcmpi(qPlotLabel, 'hip_flexion_r')
        %    data = data + 12;
        %elseif any(strcmpi(qPlotLabel, {'knee_angle_r_ik_moment' 'knee_angle_r_rra1_moment' 'knee_angle_r_rra2_moment' 'knee_angle_r_cmc_moment'}))
        %    data = -1*data;
        % I'm not sure if this is right either: (Chand)
        %elseif strcmpi(qPlotLabel, 'hip_flexion_l')
        %    data = data + 12;
        %elseif any(strcmpi(qPlotLabel, {'knee_angle_l_ik_moment' 'knee_angle_l_rra1_moment' 'knee_angle_l_rra2_moment' 'knee_angle_l_cmc_moment'}))
        %    data = -1*data;
        %end

        % Plot data vs time.
        hold on;
        p = plot(time, data);
        set(p, 'LineStyle', pLineStyle{curveNum}, 'LineWidth', ...
                pLineWidth{curveNum}, 'Color', pLineColor{curveNum});
    end    
end
return;
