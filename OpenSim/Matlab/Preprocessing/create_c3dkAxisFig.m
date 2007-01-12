function [] = create_c3dkAxisFig(measuredAngles, correctedAngles, g, ...
                                                vTime, gTimes, figHandle)
% Purpose:  Generates plots of knee flex/ext, knee var/val, knee rotation,
%           and hip rotation angles vs time, before and after correction 
%           for a malaligned knee axis, for a simulateable segment of data 
%           read from the C3D file of a Gillette subject, for the limb of 
%           interest. 
%
% Input:    measuredAngles is a structure comprised of the following:
%               *.kf - array of 'measured' knee flex/ext angles
%                .kv - array of 'measured' knee var/val angles
%                .kr - array of 'measured' knee rotation angles
%                .hr - array of 'measured' hip rotation angles
%           correctedAngles returns a structure comprised of the following:
%               *.kf - array of 'corrected' knee flex/ext angles
%                .kv - array of 'corrected' knee var/val angles
%                .kr - array of 'corrected' knee rotation angles
%                .hr - array of 'corrected' hip rotation angles
%                .offset - kAxis offset (rotation about Z axis)
%                               used for correction
%           g is a structure read from read_gcdMean()
%           vTime is an array of time values corresponding to 
%               measuredAngles and correctedAngles 
%           gTimes is a matrix(nPts, nCycles) corresponding to g
%               for the limb of interest
%           figHandle is the handle number of the new figure window
%
% Called Functions:
%       ref_c3dkAxisPlotLabels(source)
%       create_c3dkAxisPlotTemplate(jntangles, timeMatrix, dataIndex)
%       format_c3dkAxisPlotTemplate(plotIndex, vTime)
%       overlay_c3dkAxisData(data, vTime, source)
%
% ASA, 10-05


% Specify attributes of figure window.
nPlotRows = 2;                      
nPlotCols = 3;
nSubPlots = nPlotRows * nPlotCols;  
set(0, 'Units', 'pixels');          
scnsize = get(0, 'ScreenSize');
scnwidth = scnsize(3);
scnheight = scnsize(4);
figPos = [0.15*scnwidth, 0.05*scnheight, 0.8*scnwidth, 0.8*scnheight];
figColor = 'w';

% Generate new figure window.
figure(figHandle);     
clf
set(gcf, 'Position', figPos, 'Color', figColor);

% Generate subplot templates and plot averaged data from control subjects.
plotLabels = ref_c3dkAxisPlotLabels('template');  
for jntangleIndex = 1:length(g.jntangles)          
    gLabels{jntangleIndex} = g.jntangles(jntangleIndex).label;
end
for plotIndex = 1:nSubPlots
    subplot(nPlotRows, nPlotCols, plotIndex);
    if plotIndex > 2      
        dataIndex = strmatch(plotLabels{plotIndex}, gLabels);
        create_c3dkAxisPlotTemplate(g.jntangles, gTimes, dataIndex);
        format_c3dkAxisPlotTemplate(plotIndex, vTime);
    end
end

% Overlay measured and corrected angles.
plotLabels = ref_c3dkAxisPlotLabels('data');  
for plotIndex = 1:nSubPlots
    subplot(nPlotRows, nPlotCols, plotIndex);
    if plotIndex > 2     
        data = eval(['measuredAngles.', plotLabels{plotIndex}]);
        overlay_c3dkAxisData(data, vTime, 'measured');
        data = eval(['correctedAngles.', plotLabels{plotIndex}]);
        overlay_c3dkAxisData(data, vTime, 'corrected');
    end
end
        
% Add legend.
offsetString = sprintf('%4.1f', correctedAngles.offset);
legendString{1} = 'measured, solid black';
legendString{2} = 'corrected, dashed cyan';
legendString{3} = strcat({'knee axis offset = '}, offsetString);
subplot(nPlotRows, nPlotCols, 1);
for i = 1:length(legendString)
    x = 0;
    y = 1 - 0.10*i;
    text(x, y, legendString{i}, 'FontSize', 8);
end
axis off;

subplot(nPlotRows, nPlotCols, 2);
axis off;
return;
