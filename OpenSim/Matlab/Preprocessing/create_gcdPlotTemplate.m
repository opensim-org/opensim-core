function [] = create_gcdPlotTemplate(cycle, s, index)
% Purpose:  Adds averaged data from Gillette control subjects to the 
%           current subplot created by create_gcdFigTemplate() or by
%           create_jntAngMomFigTemplate().
%
% Input:    cycle is an array of % gait cycle values
%           s is one the following sub-structures of g = read_gcdMean():
%               s(nAngles)  .label, .ave, .sd     = g.jntangles()
%               s(nMoments) .label, .ave, .sd     = g.jntmoments()
%           index specifies which element of s to plot
%               (i.e., which joint angle or joint moment to plot)
%           
% ASA, 6-05


% Specify attributes of subplot.
zeroLine = zeros(51, 1);
pLineStyle = ':';
pLineWidth = 0.5;
pLineColor = [0.3, 0.3, 0.3];
  
% Add averaged control data +/- 1SD and +/- 2SD to the current subplot.
p = plot(cycle, s(index).ave);
        set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
               'Color', pLineColor);
        hold on;
p = plot(cycle, s(index).ave + s(index).sd);
        set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
               'Color', pLineColor);
        hold on;
p = plot(cycle, s(index).ave - s(index).sd);
        set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
               'Color', pLineColor);
        hold on;
p = plot(cycle, s(index).ave + 2*s(index).sd);
        set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
               'Color', pLineColor);
        hold on;
p = plot(cycle, s(index).ave - 2*s(index).sd);
        set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
               'Color', pLineColor);
        hold on;
p = plot(cycle, zeroLine);
        set(p, 'LineWidth', pLineWidth, 'Color', pLineColor);
return;