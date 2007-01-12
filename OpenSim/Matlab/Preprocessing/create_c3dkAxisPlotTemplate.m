function [] = create_c3dkAxisPlotTemplate(jntangles, timeMatrix, dataIndex)
% Purpose:  Adds averaged data from control subjects to the current 
%           subplot created by create_c3dkAxisFig()
%
% Input:    jntangles is the jntangles element of g = read_gcdMean() 
%               with the following format:
%                       *(nAngles)   .label, .ave, .sd   
%           timeMatrix is a matrix of time values (nPts, nCycles) 
%               corresponding to gait cycles in a 'simulateable' segment,
%               for the current limb of interest
%           dataIndex specifies which element of jntangles to plot
% 
% ASA, 7-05, rev 10-05 for C3D file


% Specify attributes of subplot.
pLineStyle = ':';
pLineWidth = 0.5;
pLineColor = [0.3, 0.3, 0.3];
  
% Get relevant dimensions.
[nPts, nCycles] = size(timeMatrix);

% Add averaged control data +/- 1SD and +/- 2SD to the current subplot.
for cycleNum = 1:nCycles
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
            hold on;
end
return;
              
