function [] = overlay_c3dkAxisData(data, vTime, source)
% Purpose:  Overlays 'measured' or 'corrected' data onto the current
%           subplot created by create_c3dkAxisFig()
%
% Input:    data is an array of joint angle values
%           vTime is an array of time values corresponding to data
%           source is used to select a line type and style;
%             it specifies whether the data being plotted are:
%               'measured' -  read from C3D file
%               'corrected' - corrected for malalignment of the knee axis
%          
% ASA, 7-05, rev 10-05 for C3D file


% Specify attributes of plot curves to be generated.
if strcmpi(source, 'measured')
    pLineStyle = '-';
    pLineWidth = 1.5;
    pLineColor = 'k';
elseif strcmpi(source, 'corrected')
    pLineStyle = '--';
    pLineWidth = 1.5;
    pLineColor = 'c';        
end

hold on;
p = plot(vTime, data);
set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
               'Color', pLineColor);
return;
