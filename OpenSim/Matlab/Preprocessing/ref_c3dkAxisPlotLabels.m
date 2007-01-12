function plotLabel = ref_c3dkAxisPlotLabels(source)
% Purpose:  Returns a list of labels corresponding to subplots of a 
%           figure window created by create_c3dkAxisFig().
%
% Input:    source has 2 possible values:
%           'template' - for accessing data read from read_gcdMean()
%           'data' - for accessing data corresponding to a subject's
%                    measured or corrected joint angles
%           
% Output:   plotLabel{} returns a cell array containing the list of labels
% ASA, 10-05


if strcmpi(source, 'template')
    plotLabel{1}  = {};
    plotLabel{2}  = {};
    plotLabel{3}  = 'HipRotation';
    plotLabel{4}  = 'KneeValgVar';
    plotLabel{5}  = 'KneeFlexExt';
    plotLabel{6}  = 'KneeRotation';
    
elseif strcmpi(source, 'data')
    plotLabel{1}  = {};
    plotLabel{2}  = {};
    plotLabel{3}  = 'hr';
    plotLabel{4}  = 'kv';
    plotLabel{5}  = 'kf';
    plotLabel{6}  = 'kr';    
end
