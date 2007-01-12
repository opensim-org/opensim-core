function [] = create_jntAngMomFigTemplate(figHandle, g)
% Purpose:  Creates a new figure window, Figure(figHandle), and 
%           generates a template for plotting joint angles and 
%           joint moments vs gait cycle.
%
%           Averaged control data +/- 1 SD and +/- 2SD are plotted,
%           as read from g.
%
% Input:    figHandle is the handle number of the new figure window
%           g is a structure containing averaged control data
%               as read from g = read_gcdMean()
%
% Called Functions:
%           ref_gcdPlotLabels(source)
%           create_gcdPlotTemplate(cycle, s, index)
%           format_gcdPlotTemplate(plotIndex)
%
% ASA, 6-05


% Specify attributes of figure window.
nPlotRows = 5;                      
nPlotCols = 3;
nSubPlots = nPlotRows * nPlotCols;  
set(0, 'Units', 'pixels');          
scnsize = get(0, 'ScreenSize');
scnwidth = scnsize(3);
scnheight = scnsize(4);
figPos = [0.15*scnwidth, 0.05*scnheight, 0.65*scnwidth, 0.8*scnheight];
figColor = 'w';

% Generate figure window.
figure(figHandle);     
clf
set(gcf, 'Position', figPos, 'Color', figColor);

% Specify contents of subplots.
plotLabel = ref_gcdPlotLabels('template');

% Get labels corresponding to joint angles and joint moments stored in g. 
for jntangleIndex = 1:length(g.jntangles)
    jntangleLabels{jntangleIndex} = g.jntangles(jntangleIndex).label;
end
for jntmomentIndex = 1:length(g.jntmoments)
    jntmomentLabels{jntmomentIndex} = g.jntmoments(jntmomentIndex).label;
end

% Plot averaged data from control subjects.
for plotIndex = 1:nSubPlots
    if plotIndex ~= 10          % reserve subplot 10 for legend
        subplot(nPlotRows, nPlotCols, plotIndex);
        if plotIndex < 13       % plots of joint angles
            dataIndex = strmatch(plotLabel{plotIndex}, jntangleLabels);
            create_gcdPlotTemplate(g.cycle, g.jntangles, dataIndex);
        else                    % plots of joint moments
            dataIndex = strmatch(plotLabel{plotIndex}, jntmomentLabels);
            create_gcdPlotTemplate(g.cycle, g.jntmoments, dataIndex);
        end
        format_gcdPlotTemplate(plotIndex);
    end
end
return;