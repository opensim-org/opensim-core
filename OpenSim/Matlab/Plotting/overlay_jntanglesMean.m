function [] = overlay_jntanglesMean(jntangles, gcdMeanLabel, timeMatrix)
% Purpose:  Extracts the data specified by gcdMeanLabel from the structure
%           jntangles, and plots the mean data +/- 1SD, 2SD vs time 
%           on the current subplot.
%
% Input:    jntangles is a sub-structure read from read_gcdMean(), 
%             formatted as follows:
%               jntangles()  .label, .ave, .sd 
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

    % Get index of joint angle to plot.
    for jntangleIndex = 1:length(jntangles)
        jntangleLabels{jntangleIndex} = jntangles(jntangleIndex).label;
    end
    dataIndex = strmatch(gcdMeanLabel, jntangleLabels);
    
    % Get relevant dimensions.
    [nPts, nCycles] = size(timeMatrix);

    % Add averaged control data +/- 1SD and +/- 2SD to the current subplot.
    for cycleNum = 1:nCycles
        hold on;
        p = plot(timeMatrix(:, cycleNum), jntangles(dataIndex).ave);
            set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                   'Color', pLineColor);
        hold on;
        p = plot(timeMatrix(:, cycleNum), ...
                jntangles(dataIndex).ave + jntangles(dataIndex).sd);
            set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                   'Color', pLineColor);
        hold on;
        p = plot(timeMatrix(:, cycleNum), ...
                jntangles(dataIndex).ave - jntangles(dataIndex).sd);
            set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                   'Color', pLineColor);
        hold on;
        p = plot(timeMatrix(:, cycleNum), ...
                jntangles(dataIndex).ave + 2*jntangles(dataIndex).sd);
            set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                   'Color', pLineColor);
        hold on;
        p = plot(timeMatrix(:, cycleNum), ...
                jntangles(dataIndex).ave - 2*jntangles(dataIndex).sd);
            set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
                   'Color', pLineColor);
    end
end
return;
