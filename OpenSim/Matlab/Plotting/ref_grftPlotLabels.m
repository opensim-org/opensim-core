function figContents = ref_grftPlotLabels()
% Purpose:  Returns a list of labels specifying the data to be plotted
%           in each subplot of each figure created by 
%           compare_grftFromMot().
%
%           NOTES:
%           qPlotLabel{} specifies column headings of a motion file 
%               associated with the UW Gait Workflow; the corresponding 
%               data will be plotted on figNum(plotNum)
%           qMeasuredLabel{} specifies column headings of a motion file 
%               associated with measured data; the corresponding 
%               data will be plotted on figNum(plotNum)
%           subplotTitle{} specifies title of each subplot
%           subplotAxisLabel{} specifies y-axis label of each subplot
%           subplotRange{} specifies the min, max values for scaling each
%               subplot
%           subplotTick{} specifies the y-tick labels of each subplot
%
% Output:   figContents returns a structure, formatted as follows:
%               *(figNum).qPlotLabel{plotNum}                                
%               *(figNum).qMeasuredLabel{plotNum} 
%               *(figNum).subplotTitle{plotNum}   
%               *(figNum).subplotAxisLabel{plotNum}   
%               *(figNum).subplotRange{plotNum}   
%               *(figNum).subplotTick{plotNum}
%
% ASA, 11-05, rev 2-06


% Figure 1:  R GRF Data.
qPlotLabel{1} = 'ground_force_vx';
qPlotLabel{2} = {};
qPlotLabel{3} = 'ground_force_vy';
qPlotLabel{4} = 'ground_force_vy';
qPlotLabel{5} = 'ground_force_vz';
qPlotLabel{6} = 'ground_force_px';
qPlotLabel{7} = 'ground_torque_y';
qPlotLabel{8} = 'ground_force_pz';
qMeasuredLabel{1} = 'ground_force_vx';
qMeasuredLabel{2} = {};
qMeasuredLabel{3} = 'ground_force_vy';
qMeasuredLabel{4} = 'ground_force_vy';
qMeasuredLabel{5} = 'ground_force_vz';
qMeasuredLabel{6} = 'ground_force_px';
qMeasuredLabel{7} = 'ground_torque_y';
qMeasuredLabel{8} = 'ground_force_pz';
subplotTitle{1} = {'R fore-aft GRF'};
subplotTitle{2} = {};
subplotTitle{3} = {'R vertical GRF'};
subplotTitle{4} = {'R vertical GRF'};
subplotTitle{5} = {'R med-lat GRF'};
subplotTitle{6} = {'R fore-aft COP'};
subplotTitle{7} = {'R vertical torque'};
subplotTitle{8} = {'R med-lat COP'};
subplotAxisLabel{1} = '(N)';
subplotAxisLabel{2} = {};
subplotAxisLabel{3} = '(N)';
subplotAxisLabel{4} = '(m)';
subplotAxisLabel{5} = '(N)';
subplotAxisLabel{6} = '(m)';
subplotAxisLabel{7} = '(Nm)';
subplotAxisLabel{8} = '(m)';
subplotRange{1} = {};
subplotRange{2} = {};
subplotRange{3} = {};
subplotRange{4} = [-5 2];
subplotRange{5} = {};
subplotRange{6} = {};
subplotRange{7} = {};
subplotRange{8} = {};
subplotTick{1}  = {};
subplotTick{2}  = {};
subplotTick{3}  = {};
subplotTick{4}  = [-5:5:5];
subplotTick{5}  = {};
subplotTick{6}  = {};
subplotTick{7}  = {};
subplotTick{8}  = {};
figContents(1).qPlotLabel = qPlotLabel;
figContents(1).qMeasuredLabel = qMeasuredLabel;
figContents(1).subplotTitle = subplotTitle;
figContents(1).subplotAxisLabel = subplotAxisLabel;
figContents(1).subplotRange = subplotRange;
figContents(1).subplotTick = subplotTick;
clear qPlotLabel qMeasuredLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;

% Figure 2:  L GRF Data.
qPlotLabel{1} = 'ground_force_vx';
qPlotLabel{2} = {};
qPlotLabel{3} = 'ground_force_vy';
qPlotLabel{4} = 'ground_force_vy';
qPlotLabel{5} = 'ground_force_vz';
qPlotLabel{6} = 'ground_force_px';
qPlotLabel{7} = 'ground_torque_y';
qPlotLabel{8} = 'ground_force_pz';
qMeasuredLabel{1} = 'ground_force_vx';
qMeasuredLabel{2} = {};
qMeasuredLabel{3} = 'ground_force_vy';
qMeasuredLabel{4} = 'ground_force_vy';
qMeasuredLabel{5} = 'ground_force_vz';
qMeasuredLabel{6} = 'ground_force_px';
qMeasuredLabel{7} = 'ground_torque_y';
qMeasuredLabel{8} = 'ground_force_pz';
subplotTitle{1} = {'L fore-aft GRF'};
subplotTitle{2} = {};
subplotTitle{3} = {'L vertical GRF'};
subplotTitle{4} = {'L vertical GRF'};
subplotTitle{5} = {'L med-lat GRF'};
subplotTitle{6} = {'L fore-aft COP'};
subplotTitle{7} = {'L vertical torque'};
subplotTitle{8} = {'L med-lat COP'};
subplotAxisLabel{1} = '(N)';
subplotAxisLabel{2} = {};
subplotAxisLabel{3} = '(N)';
subplotAxisLabel{4} = '(m)';
subplotAxisLabel{5} = '(N)';
subplotAxisLabel{6} = '(m)';
subplotAxisLabel{7} = '(Nm)';
subplotAxisLabel{8} = '(m)';
subplotRange{1} = {};
subplotRange{2} = {};
subplotRange{3} = {};
subplotRange{4} = [-5 2];
subplotRange{5} = {};
subplotRange{6} = {};
subplotRange{7} = {};
subplotRange{8} = {};
subplotTick{1}  = {};
subplotTick{2}  = {};
subplotTick{3}  = {};
subplotTick{4}  = [-5:5:5];
subplotTick{5}  = {};
subplotTick{6}  = {};
subplotTick{7}  = {};
subplotTick{8}  = {};
figContents(2).qPlotLabel = qPlotLabel;
figContents(2).qMeasuredLabel = qMeasuredLabel;
figContents(2).subplotTitle = subplotTitle;
figContents(2).subplotAxisLabel = subplotAxisLabel;
figContents(2).subplotRange = subplotRange;
figContents(2).subplotTick = subplotTick;
clear qPlotLabel qMeasuredLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;
return;