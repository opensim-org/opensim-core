function figContents = ref_jntmomentSagittalPlotLabels()
% Purpose:  Returns a list of labels specifying the data to be plotted
%           in each subplot of each figure created by 
%           compare_jntmomentsSagittal(). 
%
%           NOTES:
%           qPlotLabel{} specifies column headings of a motion file 
%               associated with the UW Gait Workflow; the corresponding 
%               data will be plotted on figNum(plotNum)
%           qMeasuredLabel{} specifies column headings of a motion file 
%               associated with measured data; the corresponding 
%               data will be plotted on figNum(plotNum)
%           gcdMeanLabel{} specifies column headings of the file
%               gilletteMeanSD.GCD, read from read_gcdMean().
%           subplotTitle{} specifies title of each subplot
%           subplotAxisLabel{} specifies y-axis label of each subplot
%           subplotRange{} specifies the min, max values for scaling each
%               subplot
%           subplotTick{} specifies the y-tick labels of each subplot
%
% Output:   figContents returns a structure, formatted as follows:
%               *(figNum).qPlotLabel{plotNum}                                
%               *(figNum).qMeasuredLabel{plotNum} 
%               *(figNum).gcdMeanLabel{plotNum}              
%               *(figNum).subplotTitle{plotNum}   
%               *(figNum).subplotAxisLabel{plotNum}   
%               *(figNum).subplotRange{plotNum}   
%               *(figNum).subplotTick{plotNum}
%
% ASA, 11-05, rev 2-06


qPlotLabel{1} = 'hip_flexion_l_torque';
qPlotLabel{2} = 'hip_flexion_r_torque';
qPlotLabel{3} = 'knee_angle_l_torque';
qPlotLabel{4} = 'knee_angle_r_torque';
qPlotLabel{5} = 'ankle_angle_l_torque';
qPlotLabel{6} = 'ankle_angle_r_torque';
qPlotLabel{7} = {};
qPlotLabel{8} = {};
qMeasuredLabel{1} = 'LHipMoment';
qMeasuredLabel{2} = 'RHipMoment';
qMeasuredLabel{3} = 'LKneeMoment';
qMeasuredLabel{4} = 'RKneeMoment';
qMeasuredLabel{5} = 'LAnkleMoment';
qMeasuredLabel{6} = 'RAnkleMoment';
qMeasuredLabel{7} = {};
qMeasuredLabel{8} = {};
gcdMeanLabel{1} = 'HipFlexExtMoment';
gcdMeanLabel{2} = 'HipFlexExtMoment';
gcdMeanLabel{3} = 'KneeFlexExtMoment';
gcdMeanLabel{4} = 'KneeFlexExtMoment';
gcdMeanLabel{5} = 'DorsiPlanFlexMoment';
gcdMeanLabel{6} = 'DorsiPlanFlexMoment';
gcdMeanLabel{7} = {};
gcdMeanLabel{8} = {};
subplotTitle{1} = 'L hip flex/extension+ moment';
subplotTitle{2} = 'R hip flex/extension+ moment';
subplotTitle{3} = 'L knee flex/extension+ moment';
subplotTitle{4} = 'R knee flex/extension+ moment';
subplotTitle{5} = 'L ankle dorsi/plantarflexion+ moment';
subplotTitle{6} = 'R ankle dorsi/plantarflexion+ moment';
subplotTitle{7} = {};
subplotTitle{8} = {};
subplotAxisLabel{1} = '(Nm/kg)';
subplotAxisLabel{2} = {};
subplotAxisLabel{3} = '(Nm/kg)';
subplotAxisLabel{4} = {};
subplotAxisLabel{5} = '(Nm/kg)';
subplotAxisLabel{6} = {};
subplotAxisLabel{7} = {};
subplotAxisLabel{8} = {};
subplotRange{1} = [-1.5 1.5];
subplotRange{2} = [-1.5 1.5];
subplotRange{3} = [-1.5 1.5];
subplotRange{4} = [-1.5 1.5];
subplotRange{5} = [-0.5 2.25];
subplotRange{6} = [-0.5 2.25];
subplotRange{7} = {};
subplotRange{8} = {};
subplotTick{1}  = [-1:1:2];
subplotTick{2}  = [-1:1:2];
subplotTick{3}  = [-1:1:2];
subplotTick{4}  = [-1:1:2];
subplotTick{5}  = [-1:1:2];
subplotTick{6}  = [-1:1:2];
subplotTick{7}  = {};
subplotTick{8}  = {};
figContents(1).qPlotLabel = qPlotLabel;
figContents(1).qMeasuredLabel = qMeasuredLabel;
figContents(1).gcdMeanLabel = gcdMeanLabel;
figContents(1).subplotTitle = subplotTitle;
figContents(1).subplotAxisLabel = subplotAxisLabel;
figContents(1).subplotRange = subplotRange;
figContents(1).subplotTick = subplotTick;
clear qPlotLabel qMeasuredLabel gcdMeanLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;
return;