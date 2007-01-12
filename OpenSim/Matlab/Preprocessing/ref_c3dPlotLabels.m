function [plotLabel, plotDataColumn] = ref_c3dPlotLabels(source)
% Purpose:  Returns a list of labels and data indices, as defined in the 
%           C3D files of Gillette control subjects, corresponding to 
%           subplots of a template figure window created by
%           create_jntAngMomFigTemplate().
%
% Input:    source has 2 possible values:
%           'L' - for accessing L labels/data 
%           'R' - for accessing R labels/data 
%           
% Output:   plotLabel{subPlotNum} returns a cell array of labels
%               corresponding to each subplot
%           plotDataColumn(subPlotNum) returns an array of column indices
%               corresponding to each subplot and label
%
% ASA, 7-05
% revised 9-05

if strcmpi(source, 'L')
    plotLabel{1}  = 'LPelvisAngles';                % pelvic obliquity
        plotDataColumn(1) = 2;
    plotLabel{2}  = 'LPelvisAngles';                % pelvic tilt
        plotDataColumn(2) = 1;
    plotLabel{3}  = 'LPelvisAngles';                % pelvic rotation
        plotDataColumn(3) = 3;
    plotLabel{4}  = 'LHipAngles';                   % hip ab/adduction
        plotDataColumn(4) = 2;
    plotLabel{5}  = 'LHipAngles';                   % hip flex/extension
        plotDataColumn(5) = 1;
    plotLabel{6}  = 'LHipAngles';                   % hip rotation
        plotDataColumn(6) = 3;
    plotLabel{7}  = 'LKneeAngles';                  % knee varus/valgus
        plotDataColumn(7) = 2;
    plotLabel{8}  = 'LKneeAngles';                  % knee flex/extension
        plotDataColumn(8) = 1;
    plotLabel{9}  = 'LKneeAngles';                  % knee rotation
        plotDataColumn(9) = 3;
    plotLabel{10} = {};                             % empty for legend
        plotDataColumn(10) = 0;
    plotLabel{11} = 'LAnkleAngles';                 % ankle plantarflexion
        plotDataColumn(11) = 1;
    plotLabel{12} = 'LFootProgressAngles';          % foot progression
        plotDataColumn(12) = 3;
    plotLabel{13} = 'LHipMoment';                   % hip flex/ext moment
        plotDataColumn(13) = 1;
    plotLabel{14} = 'LKneeMoment';                  % knee flex/ext moment
        plotDataColumn(14) = 1;
    plotLabel{15} = 'LAnkleMoment';                 % ankle flex/ext moment
        plotDataColumn(15) = 1;
        
elseif strcmpi(source, 'R')
    plotLabel{1}  = 'RPelvisAngles';                % pelvic obliquity
        plotDataColumn(1) = 2;
    plotLabel{2}  = 'RPelvisAngles';                % pelvic tilt
        plotDataColumn(2) = 1;
    plotLabel{3}  = 'RPelvisAngles';                % pelvic rotation
        plotDataColumn(3) = 3;
    plotLabel{4}  = 'RHipAngles';                   % hip ab/adduction
        plotDataColumn(4) = 2;
    plotLabel{5}  = 'RHipAngles';                   % hip flex/extension
        plotDataColumn(5) = 1;
    plotLabel{6}  = 'RHipAngles';                   % hip rotation
        plotDataColumn(6) = 3;
    plotLabel{7}  = 'RKneeAngles';                  % knee varus/valgus
        plotDataColumn(7) = 2;
    plotLabel{8}  = 'RKneeAngles';                  % knee flex/extension
        plotDataColumn(8) = 1;
    plotLabel{9}  = 'RKneeAngles';                  % knee rotation
        plotDataColumn(9) = 3;
    plotLabel{10} = {};                             % empty for legend
        plotDataColumn(10) = 0;
    plotLabel{11} = 'RAnkleAngles';                 % ankle plantarflexion
        plotDataColumn(11) = 1;    
    plotLabel{12} = 'RFootProgressAngles';          % foot progression
        plotDataColumn(12) = 3;
    plotLabel{13} = 'RHipMoment';                   % hip flex/ext moment
        plotDataColumn(13) = 1;
    plotLabel{14} = 'RKneeMoment';                  % knee flex/ext moment
        plotDataColumn(14) = 1;
    plotLabel{15} = 'RAnkleMoment';                 % ankle flex/ext moment
        plotDataColumn(15) = 1;
end
