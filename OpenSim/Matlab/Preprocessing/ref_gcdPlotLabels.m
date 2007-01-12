function plotLabel = ref_gcdPlotLabels(source)
% Purpose:  Returns a list of labels, as defined in GCD files, 
%           corresponding to subplots of a template figure window 
%           created by create_gcdFigTemplate().
%
% Input:    source has 3 possible values:
%               'template' - for accessing data read from read_gcdMean()
%               'L' - for accessing L data read from read_gcdFile()
%               'R' - for accessing R data read from read_gcdFile()
%           
% Output:   plotLabel{} returns a cell array containing the list of labels
% ASA, 6-05


if strcmpi(source, 'template')
    plotLabel{1}  = 'PelvicObliquity';
    plotLabel{2}  = 'PelvicTilt';
    plotLabel{3}  = 'PelvicRotation';
    plotLabel{4}  = 'HipAbAdduct';
    plotLabel{5}  = 'HipFlexExt';
    plotLabel{6}  = 'HipRotation';
    plotLabel{7}  = 'KneeValgVar';
    plotLabel{8}  = 'KneeFlexExt';
    plotLabel{9}  = 'KneeRotation';
    plotLabel{10} = {};
    plotLabel{11} = 'DorsiPlanFlex';
    plotLabel{12} = 'FootProgression';
    plotLabel{13} = 'HipFlexExtMoment';
    plotLabel{14} = 'KneeFlexExtMoment';
    plotLabel{15} = 'DorsiPlanFlexMoment';

elseif strcmpi(source, 'L')
    plotLabel{1}  = 'LeftPelvicObliquity';
    plotLabel{2}  = 'LeftPelvicTilt';
    plotLabel{3}  = 'LeftPelvicRotation';
    plotLabel{4}  = 'LeftHipAbAdduct';
    plotLabel{5}  = 'LeftHipFlexExt';
    plotLabel{6}  = 'LeftHipRotation';
    plotLabel{7}  = 'LeftKneeValgVar';
    plotLabel{8}  = 'LeftKneeFlexExt';
    plotLabel{9}  = 'LeftKneeRotation';
    plotLabel{10} = {};
    plotLabel{11} = 'LeftDorsiPlanFlex';
    plotLabel{12} = 'LeftFootProgression';
    plotLabel{13} = 'LeftHipFlexExtMoment';
    plotLabel{14} = 'LeftKneeFlexExtMoment';
    plotLabel{15} = 'LeftDorsiPlanFlexMoment';

elseif strcmpi(source, 'R')
    plotLabel{1}  = 'RightPelvicObliquity';
    plotLabel{2}  = 'RightPelvicTilt';
    plotLabel{3}  = 'RightPelvicRotation';
    plotLabel{4}  = 'RightHipAbAdduct';
    plotLabel{5}  = 'RightHipFlexExt';
    plotLabel{6}  = 'RightHipRotation';
    plotLabel{7}  = 'RightKneeValgVar';
    plotLabel{8}  = 'RightKneeFlexExt';
    plotLabel{9}  = 'RightKneeRotation';
    plotLabel{10} = {};
    plotLabel{11} = 'RightDorsiPlanFlex';
    plotLabel{12} = 'RightFootProgression';
    plotLabel{13} = 'RightHipFlexExtMoment';
    plotLabel{14} = 'RightKneeFlexExtMoment';
    plotLabel{15} = 'RightDorsiPlanFlexMoment';
end
