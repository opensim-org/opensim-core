function [] = overlay_jntmomentsMean(jntmoments, gcdMeanLabel, timeMatrix)
% Purpose:  Extracts the data specified by gcdMeanLabel from the structure
%           jntmoments, and plots the mean data +/- 1SD, 2SD vs time 
%           on the current subplot.
%
% Input:    jntmoments is a sub-structure read from read_gcdMean(), 
%             formatted as follows:
%               jntmoments()  .label, .ave, .sd 
%           gcdMeanLabel specifies the label of the data to be plotted
%           timeMatrix is a matrix of time values (nPts, nCycles) 
%               corresponding to gait cycles in a 'simulateable' segment,
%               for the current limb of interest
%
% ASA, 11-05


% Specify attributes of subplots.
pLineStyle = ':';
pLineWidth = 0.5;
pLineColor = [0.3, 0.3, 0.3];

% Get data corresponding to plot label.
if ~isempty(gcdMeanLabel)
    
    % Get index of joint moment to plot.
    for jntmomentIndex = 1:length(jntmoments)
        jntmomentLabels{jntmomentIndex} = jntmoments(jntmomentIndex).label;
    end
    dataIndex = strmatch(gcdMeanLabel, jntmomentLabels);
    
    % Get relevant dimensions.
    [nPts, nCycles] = size(timeMatrix);

    % Add averaged control data +/- 1SD and +/- 2SD to the current subplot.
    for cycleNum = 1:nCycles
        p = plot(timeMatrix(:, cycleNum), jntmoments(dataIndex).ave);
            set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                   'Color', pLineColor);
            hold on;
        p = plot(timeMatrix(:, cycleNum), ...
                jntmoments(dataIndex).ave + jntmoments(dataIndex).sd);
            set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                   'Color', pLineColor);
            hold on;
        p = plot(timeMatrix(:, cycleNum), ...
                jntmoments(dataIndex).ave - jntmoments(dataIndex).sd);
            set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                   'Color', pLineColor);
            hold on;
        p = plot(timeMatrix(:, cycleNum), ...
                jntmoments(dataIndex).ave + 2*jntmoments(dataIndex).sd);
            set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                   'Color', pLineColor);
            hold on;
        p = plot(timeMatrix(:, cycleNum), ...
                jntmoments(dataIndex).ave - 2*jntmoments(dataIndex).sd);
            set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                   'Color', pLineColor);
            hold on;
    end
end
return;