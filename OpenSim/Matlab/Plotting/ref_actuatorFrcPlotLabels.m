function figContents = ref_actuatorFrcPlotLabels()
% Purpose:  Returns a list of labels specifying the data to be plotted
%           in each subplot of each figure created by 
%           compare_actuatorFrcFromMot(). 
%
%           NOTES:
%           qPlotLabel{} specifies column headings of a motion file 
%               associated with the SimTrack Workflow; the corresponding 
%               data will be plotted on figNum(plotNum)
%           subplotTitle{} specifies title of each subplot
%           subplotAxisLabel{} specifies y-axis label of each subplot
%           subplotRange{} specifies the min, max values for scaling each
%               subplot
%           subplotTick{} specifies the y-tick labels of each subplot
%
% Output:   figContents returns a structure, formatted as follows:
%               *(figNum).qPlotLabel{plotNum}                                
%               *(figNum).subplotTitle{plotNum}   
%               *(figNum).subplotAxisLabel{plotNum}   
%               *(figNum).subplotRange{plotNum}   
%               *(figNum).subplotTick{plotNum}
%
% CTJ, 02-07, adapted from:
% ASA, 11-05, rev 2-06


% Figure 1: Residual actuators.
qPlotLabel{1}  = {'ground_force_vy'};
qPlotLabel{2}  = {'ground_force_vy'};
qPlotLabel{3}  = {};
qPlotLabel{4}  = {};
qPlotLabel{5}  = {'FX_frc'};
qPlotLabel{6}  = {'MX_frc'};
qPlotLabel{7}  = {'FY_frc'};
qPlotLabel{8}  = {'MY_frc'};
qPlotLabel{9}  = {'FZ_frc'};
qPlotLabel{10} = {'MZ_frc'};
subplotTitle{1}  = {'Left'};
subplotTitle{2}  = {'Right'};
subplotTitle{3}  = {};
subplotTitle{4}  = {};
subplotTitle{5}  = {'FX'};
subplotTitle{6}  = {'MX'};
subplotTitle{7}  = {'FY'};
subplotTitle{8}  = {'MY'};
subplotTitle{9}  = {'FZ'};
subplotTitle{10} = {'MZ'};
subplotAxisLabel{1}  = 'vertical GRF (N)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = {};
subplotAxisLabel{5}  = 'actuator force (N)';
subplotAxisLabel{6}  = 'actuator torque (N \cdot m)';
subplotAxisLabel{7}  = 'actuator force (N)';
subplotAxisLabel{8}  = 'actuator torque (N \cdot m)';
subplotAxisLabel{9}  = 'actuator force (N)';
subplotAxisLabel{10} = 'actuator torque (N \cdot m)';
subplotRange{1}  = {};
subplotRange{2}  = {};
subplotRange{3}  = {};
subplotRange{4}  = {};
subplotRange{5}  = {};     % Generic Max = 500
subplotRange{6}  = {};
subplotRange{7}  = {};
subplotRange{8}  = {};
subplotRange{9}  = {};
subplotRange{10} = {};
subplotTick{1}  = {};
subplotTick{2}  = {};
subplotTick{3}  = {};
subplotTick{4}  = {};
subplotTick{5}  = {};
subplotTick{6}  = {};
subplotTick{7}  = {};
subplotTick{8}  = {};
subplotTick{9}  = {};
subplotTick{10} = {};
figContents(1).qPlotLabel = qPlotLabel;
figContents(1).subplotTitle = subplotTitle;
figContents(1).subplotAxisLabel = subplotAxisLabel;
figContents(1).subplotRange = subplotRange;
figContents(1).subplotTick = subplotTick;
clear qPlotLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;


% Figure 2: Hip reserve actuators.
qPlotLabel{1}  = {'ground_force_vy'};
qPlotLabel{2}  = {'ground_force_vy'};
qPlotLabel{3}  = {};
qPlotLabel{4}  = {};
qPlotLabel{5}  = {'hip_flexion_l_reserve_frc'};
qPlotLabel{6}  = {'hip_flexion_r_reserve_frc'};
qPlotLabel{7}  = {'hip_adduction_l_reserve_frc'};
qPlotLabel{8}  = {'hip_adduction_r_reserve_frc'};
qPlotLabel{9}  = {'hip_rotation_l_reserve_frc'};
qPlotLabel{10} = {'hip_rotation_r_reserve_frc'};
subplotTitle{1}  = {'Left'};
subplotTitle{2}  = {'Right'};
subplotTitle{3}  = {};
subplotTitle{4}  = {};
subplotTitle{5}  = {'L.Hip.Flex.Reserve'};
subplotTitle{6}  = {'R.Hip.Flex.Reserve'};
subplotTitle{7}  = {'L.Hip.Adduct.Reserve'};
subplotTitle{8}  = {'R.Hip.Adduct.Reserve'};
subplotTitle{9}  = {'L.Hip.Rot.Reserve'};
subplotTitle{10} = {'R.Hip.Rot.Reserve'};
subplotAxisLabel{1}  = 'vertical GRF (N)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = {};
subplotAxisLabel{5}  = 'actuator torque (N \cdot m)';
subplotAxisLabel{6}  = {};
subplotAxisLabel{7}  = 'actuator torque (N \cdot m)';
subplotAxisLabel{8}  = {};
subplotAxisLabel{9}  = 'actuator torque (N \cdot m)';
subplotAxisLabel{10} = {};
subplotRange{1}  = {};
subplotRange{2}  = {};
subplotRange{3}  = {};
subplotRange{4}  = {};
subplotRange{5}  = {};     % Generic Max = 900
subplotRange{6}  = {};
subplotRange{7}  = {};
subplotRange{8}  = {};
subplotRange{9}  = {};
subplotRange{10} = {};
subplotTick{1}  = {};
subplotTick{2}  = {};
subplotTick{3}  = {};
subplotTick{4}  = {};
subplotTick{5}  = {};
subplotTick{6}  = {};
subplotTick{7}  = {};
subplotTick{8}  = {};
subplotTick{9}  = {};
subplotTick{10} = {};
figContents(2).qPlotLabel = qPlotLabel;
figContents(2).subplotTitle = subplotTitle;
figContents(2).subplotAxisLabel = subplotAxisLabel;
figContents(2).subplotRange = subplotRange;
figContents(2).subplotTick = subplotTick;
clear qPlotLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;


% Figure 3: Knee and ankle reserve actuators.
qPlotLabel{1}  = {'ground_force_vy'};
qPlotLabel{2}  = {'ground_force_vy'};
qPlotLabel{3}  = {};
qPlotLabel{4}  = {};
qPlotLabel{5}  = {'knee_angle_l_reserve_frc'};
qPlotLabel{6}  = {'knee_angle_r_reserve_frc'};
qPlotLabel{7}  = {'ankle_angle_l_reserve_frc'};
qPlotLabel{8}  = {'ankle_angle_r_reserve_frc'};
qPlotLabel{9}  = {};
qPlotLabel{10} = {};
subplotTitle{1}  = {'Left'};
subplotTitle{2}  = {'Right'};
subplotTitle{3}  = {};
subplotTitle{4}  = {};
subplotTitle{5}  = {'L.Knee.Flex.Reserve'};
subplotTitle{6}  = {'R.Knee.Flex.Reserve'};
subplotTitle{7}  = {'L.Ankle.Flex.Reserve'};
subplotTitle{8}  = {'R.Ankle.Flex.Reserve'};
subplotTitle{9}  = {};
subplotTitle{10} = {};
subplotAxisLabel{1}  = 'vertical GRF (N)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = {};
subplotAxisLabel{5}  = 'actuator torque (N \cdot m)';
subplotAxisLabel{6}  = {};
subplotAxisLabel{7}  = 'actuator torque (N \cdot m)';
subplotAxisLabel{8}  = {};
subplotAxisLabel{9}  = 'actuator torque (N \cdot m)';
subplotAxisLabel{10} = {};
subplotRange{1}  = {};
subplotRange{2}  = {};
subplotRange{3}  = {};
subplotRange{4}  = {};
subplotRange{5}  = {};
subplotRange{6}  = {};
subplotRange{7}  = {};     % Generic Max = 500
subplotRange{8}  = {};
subplotRange{9}  = {};
subplotRange{10} = {};
subplotTick{1}  = {};
subplotTick{2}  = {};
subplotTick{3}  = {};
subplotTick{4}  = {};
subplotTick{5}  = {};
subplotTick{6}  = {};
subplotTick{7}  = {};
subplotTick{8}  = {};
subplotTick{9}  = {};
subplotTick{10} = {};
figContents(3).qPlotLabel = qPlotLabel;
figContents(3).subplotTitle = subplotTitle;
figContents(3).subplotAxisLabel = subplotAxisLabel;
figContents(3).subplotRange = subplotRange;
figContents(3).subplotTick = subplotTick;
clear qPlotLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;


% Figure 4: Back reserve actuators.
qPlotLabel{1}  = {'ground_force_vy'};
qPlotLabel{2}  = {'ground_force_vy'};
qPlotLabel{3}  = {};
qPlotLabel{4}  = {};
qPlotLabel{5}  = {'lumbar_extension_reserve_frc'};
qPlotLabel{6}  = {};
qPlotLabel{7}  = {'lumbar_bending_reserve_frc'};
qPlotLabel{8}  = {};
qPlotLabel{9}  = {'lumbar_rotation_reserve_frc'};
qPlotLabel{10} = {};
subplotTitle{1}  = {'Left'};
subplotTitle{2}  = {'Right'};
subplotTitle{3}  = {};
subplotTitle{4}  = {};
subplotTitle{5}  = {'Lumb.Ext.Reserve'};
subplotTitle{6}  = {};
subplotTitle{7}  = {'Lumb.Bend.Reserve'};
subplotTitle{8}  = {};
subplotTitle{9}  = {'Lumb.Rot.Reserve'};
subplotTitle{10} = {};
subplotAxisLabel{1}  = 'vertical GRF (N)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = {};
subplotAxisLabel{5}  = 'actuator torque (N \cdot m)';
subplotAxisLabel{6}  = {};
subplotAxisLabel{7}  = 'actuator torque (N \cdot m)';
subplotAxisLabel{8}  = {};
subplotAxisLabel{9}  = 'actuator torque (N \cdot m)';
subplotAxisLabel{10} = {};
subplotRange{1}  = {};
subplotRange{2}  = {};
subplotRange{3}  = {};
subplotRange{4}  = {};
subplotRange{5}  = {};     % Generic Max = 500
subplotRange{6}  = {};
subplotRange{7}  = {};
subplotRange{8}  = {};
subplotRange{9}  = {};
subplotRange{10} = {};
subplotTick{1}  = {};
subplotTick{2}  = {};
subplotTick{3}  = {};
subplotTick{4}  = {};
subplotTick{5}  = {};
subplotTick{6}  = {};
subplotTick{7}  = {};
subplotTick{8}  = {};
subplotTick{9}  = {};
subplotTick{10} = {};
figContents(4).qPlotLabel = qPlotLabel;
figContents(4).subplotTitle = subplotTitle;
figContents(4).subplotAxisLabel = subplotAxisLabel;
figContents(4).subplotRange = subplotRange;
figContents(4).subplotTick = subplotTick;
clear qPlotLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;

return;
