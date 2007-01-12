function figContents = ref_jntangleStaticPlotLabels()
% Purpose:  Returns a list of labels specifying the data to be plotted
%           in each subplot of each figure created by 
%           compare_jntanglesStatic(). 
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


% Figure 1:  Pelvis Translations, Back Angles, and Pelvis Angles
qPlotLabel{1}  = 'pelvis_ty';
qPlotLabel{2}  = 'pelvis_tx';
qPlotLabel{3}  = 'pelvis_tz';
qPlotLabel{4}  = 'lumbar_bending';
qPlotLabel{5}  = 'lumbar_extension';
qPlotLabel{6}  = 'lumbar_rotation';
qPlotLabel{7}  = 'pelvis_list';
qPlotLabel{8}  = 'pelvis_tilt';
qPlotLabel{9}  = 'pelvis_rotation';
qPlotLabel{10} = {};
qPlotLabel{11} = {};
qPlotLabel{12} = {};
qMeasuredLabel{1}  = 'pelvis_ty';
qMeasuredLabel{2}  = 'pelvis_tx';
qMeasuredLabel{3}  = 'pelvis_tz';
qMeasuredLabel{4}  = 'lumbar_bending';
qMeasuredLabel{5}  = 'lumbar_extension';
qMeasuredLabel{6}  = 'lumbar_rotation';
qMeasuredLabel{7}  = 'pelvis_list';
qMeasuredLabel{8}  = 'pelvis_tilt';
qMeasuredLabel{9}  = 'pelvis_rotation';
qMeasuredLabel{10} = {};
qMeasuredLabel{11} = {};
qMeasuredLabel{12} = {};
subplotTitle{1}  = 'pelvis translation (up+/down)';
subplotTitle{2}  = 'pelvis translation (fore+/aft)';
subplotTitle{3}  = 'pelvis translation (R+/L)';
subplotTitle{4}  = 'back bending (R down+/up)';
subplotTitle{5}  = 'back extension (post+/ant)';
subplotTitle{6}  = 'back rotation (R int+/ext)';
subplotTitle{7}  = 'pelvic obliquity (R up+/down)';
subplotTitle{8}  = 'pelvic tilt (ant+/post)';
subplotTitle{9}  = 'pelvic rotation (R int+/ext)';
subplotTitle{10} = {};
subplotTitle{11} = {};
subplotTitle{12} = {};
subplotAxisLabel{1}  = 'translation (m)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = 'joint angle (deg)';
subplotAxisLabel{5}  = {};
subplotAxisLabel{6}  = {};
subplotAxisLabel{7}  = 'joint angle (deg)';
subplotAxisLabel{8}  = {};
subplotAxisLabel{9}  = {};
subplotAxisLabel{10} = {};
subplotAxisLabel{11} = {};
subplotAxisLabel{12} = {};
subplotRange{1}  = {};
subplotRange{2}  = {};
subplotRange{3}  = {};
subplotRange{4}  = [-25 25];
subplotRange{5}  = [-35 35];
subplotRange{6}  = [-40 40];
subplotRange{7}  = [-25 25];
subplotRange{8}  = [-15 35];
subplotRange{9}  = [-40 40];
subplotRange{10} = {};
subplotRange{11} = {};
subplotRange{12} = {};
subplotTick{1}  = {};
subplotTick{2}  = {};
subplotTick{3}  = {};
subplotTick{4}  = [-20:20:20];
subplotTick{5}  = [-30:20:30];
subplotTick{6}  = [-30:20:30];
subplotTick{7}  = [-20:20:20];
subplotTick{8}  = [-10:20:30];
subplotTick{9}  = [-30:20:30];
subplotTick{10} = {};
subplotTick{11} = {};
subplotTick{12} = {};
figContents(1).qPlotLabel = qPlotLabel;
figContents(1).qMeasuredLabel = qMeasuredLabel;
figContents(1).subplotTitle = subplotTitle;
figContents(1).subplotAxisLabel = subplotAxisLabel;
figContents(1).subplotRange = subplotRange;
figContents(1).subplotTick = subplotTick;
clear qPlotLabel qMeasuredLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;

% Figure 2:  R Hip, Knee, and Ankle Angles
qPlotLabel{1}  = 'hip_adduction_r';
qPlotLabel{2}  = 'hip_flexion_r';
qPlotLabel{3}  = 'hip_rotation_r';
qPlotLabel{4}  = {};
qPlotLabel{5}  = 'knee_angle_r';
qPlotLabel{6}  = {};
qPlotLabel{7}  = 'subtalar_angle_r';
qPlotLabel{8}  = 'ankle_angle_r';
qPlotLabel{9}  = 'mtp_angle_r';
qPlotLabel{10} = {};
qPlotLabel{11} = {};
qPlotLabel{12} = {};
qMeasuredLabel{1}  = 'hip_adduction_r';
qMeasuredLabel{2}  = 'hip_flexion_r';
qMeasuredLabel{3}  = 'hip_rotation_r';
qMeasuredLabel{4}  = {};
qMeasuredLabel{5}  = 'knee_angle_r';
qMeasuredLabel{6}  = {};
qMeasuredLabel{7}  = 'subtalar_angle_r';
qMeasuredLabel{8}  = 'ankle_angle_r';
qMeasuredLabel{9}  = 'mtp_angle_r';
qMeasuredLabel{10} = {};
qMeasuredLabel{11} = {};
qMeasuredLabel{12} = {};
subplotTitle{1}  = 'R hip ad+/abduction';
subplotTitle{2}  = 'R hip flex+/extension';
subplotTitle{3}  = 'R hip int+/ext rotation';
subplotTitle{4}  = {};
subplotTitle{5}  = 'R knee flex+/extension';
subplotTitle{6}  = {};
subplotTitle{7}  = 'R subtalar inv+/eversion';
subplotTitle{8}  = 'R ankle dorsi+/plantarflexion';
subplotTitle{9}  = 'R mtp flex+/extension';
subplotTitle{10} = {};
subplotTitle{11} = {};
subplotTitle{12} = {};
subplotAxisLabel{1}  = 'joint angle (deg)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = {};
subplotAxisLabel{5}  = 'joint angle (deg)';
subplotAxisLabel{6}  = {};
subplotAxisLabel{7}  = 'joint angle (deg)';
subplotAxisLabel{8}  = {};
subplotAxisLabel{9}  = {};
subplotAxisLabel{10} = {};
subplotAxisLabel{11} = {};
subplotAxisLabel{12} = {};
subplotRange{1}  = [-25 25];
subplotRange{2}  = [-25 75];
subplotRange{3}  = [-40 40];
subplotRange{4}  = {};
subplotRange{5}  = [-15 85];
subplotRange{6}  = {};
subplotRange{7}  = [-20 20];
subplotRange{8}  = [-40 35];
subplotRange{9}  = [-20 20];
subplotRange{10} = {};
subplotRange{11} = {};
subplotRange{12} = {};
subplotTick{1}  = [-20:20:20];
subplotTick{2}  = [-15:30:75];
subplotTick{3}  = [-30:20:30];
subplotTick{4}  = {};
subplotTick{5}  = [-15:30:75];
subplotTick{6}  = {};
subplotTick{7}  = [-20:20:20];
subplotTick{8}  = [-30:20:30];
subplotTick{9}  = [-20:20:20];
subplotTick{10} = {};
subplotTick{11} = {};
subplotTick{12} = {};
figContents(2).qPlotLabel = qPlotLabel;
figContents(2).qMeasuredLabel = qMeasuredLabel;
figContents(2).subplotTitle = subplotTitle;
figContents(2).subplotAxisLabel = subplotAxisLabel;
figContents(2).subplotRange = subplotRange;
figContents(2).subplotTick = subplotTick;
clear qPlotLabel qMeasuredLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;

% Figure 3:  L Hip, Knee, and Ankle Angles
qPlotLabel{1}  = 'hip_adduction_l';
qPlotLabel{2}  = 'hip_flexion_l';
qPlotLabel{3}  = 'hip_rotation_l';
qPlotLabel{4}  = {};
qPlotLabel{5}  = 'knee_angle_l';
qPlotLabel{6}  = {};
qPlotLabel{7}  = 'subtalar_angle_l';
qPlotLabel{8}  = 'ankle_angle_l';
qPlotLabel{9}  = 'mtp_angle_l';
qPlotLabel{10} = {};
qPlotLabel{11} = {};
qPlotLabel{12} = {};
qMeasuredLabel{1}  = 'hip_adduction_l';
qMeasuredLabel{2}  = 'hip_flexion_l';
qMeasuredLabel{3}  = 'hip_rotation_l';
qMeasuredLabel{4}  = {};
qMeasuredLabel{5}  = 'knee_angle_l';
qMeasuredLabel{6}  = {};
qMeasuredLabel{7}  = 'subtalar_angle_l';
qMeasuredLabel{8}  = 'ankle_angle_l';
qMeasuredLabel{9}  = 'mtp_angle_l';
qMeasuredLabel{10} = {};
qMeasuredLabel{11} = {};
qMeasuredLabel{12} = {};
subplotTitle{1}  = 'L hip ad+/abduction';
subplotTitle{2}  = 'L hip flex+/extension';
subplotTitle{3}  = 'L hip int+/ext rotation';
subplotTitle{4}  = {};
subplotTitle{5}  = 'L knee flex+/extension';
subplotTitle{6}  = {};
subplotTitle{7}  = 'L subtalar inv+/eversion';
subplotTitle{8}  = 'L ankle dorsi+/plantarflexion';
subplotTitle{9}  = 'L mtp flex+/extension';
subplotTitle{10} = {};
subplotTitle{11} = {};
subplotTitle{12} = {};
subplotAxisLabel{1}  = 'joint angle (deg)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = {};
subplotAxisLabel{5}  = 'joint angle (deg)';
subplotAxisLabel{6}  = {};
subplotAxisLabel{7}  = 'joint angle (deg)';
subplotAxisLabel{8}  = {};
subplotAxisLabel{9}  = {};
subplotAxisLabel{10} = {};
subplotAxisLabel{11} = {};
subplotAxisLabel{12} = {};
subplotRange{1}  = [-25 25];
subplotRange{2}  = [-25 75];
subplotRange{3}  = [-40 40];
subplotRange{4}  = {};
subplotRange{5}  = [-15 85];
subplotRange{6}  = {};
subplotRange{7}  = [-20 20];
subplotRange{8}  = [-40 35];
subplotRange{9}  = [-20 20];
subplotRange{10} = {};
subplotRange{11} = {};
subplotRange{12} = {};
subplotTick{1}  = [-20:20:20];
subplotTick{2}  = [-15:30:75];
subplotTick{3}  = [-30:20:30];
subplotTick{4}  = {};
subplotTick{5}  = [-15:30:75];
subplotTick{6}  = {};
subplotTick{7}  = [-20:20:20];
subplotTick{8}  = [-30:20:30];
subplotTick{9}  = [-20:20:20];
subplotTick{10} = {};
subplotTick{11} = {};
subplotTick{12} = {};
figContents(3).qPlotLabel = qPlotLabel;
figContents(3).qMeasuredLabel = qMeasuredLabel;
figContents(3).subplotTitle = subplotTitle;
figContents(3).subplotAxisLabel = subplotAxisLabel;
figContents(3).subplotRange = subplotRange;
figContents(3).subplotTick = subplotTick;
clear qPlotLabel qMeasuredLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;

return;