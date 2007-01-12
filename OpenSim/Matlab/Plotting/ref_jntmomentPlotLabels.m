function figContents = ref_jntmomentPlotLabels()
% Purpose:  Returns a list of labels specifying the data to be plotted
%           in each subplot of each figure created by 
%           compare_jntmomentsFromMot(). 
%
%           NOTES:
%           qPlotLabel{} specifies column headings of a motion file 
%               associated with the UW Gait Workflow; the corresponding 
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
% ASA, 11-05, rev 2-06


% Figure 1:  Pelvis and Back Moments
qPlotLabel{1}  = 'pelvis_ty_torque';
qPlotLabel{2}  = 'pelvis_tx_torque';
qPlotLabel{3}  = 'pelvis_tz_torque';
qPlotLabel{4}  = 'lumbar_bending_torque';
qPlotLabel{5}  = 'lumbar_extension_torque';
qPlotLabel{6}  = 'lumbar_rotation_torque';
qPlotLabel{7}  = 'pelvis_list_torque';
qPlotLabel{8}  = 'pelvis_tilt_torque';
qPlotLabel{9}  = 'pelvis_rotation_torque';
qPlotLabel{10} = {};
qPlotLabel{11} = {};
qPlotLabel{12} = {};
subplotTitle{1}  = 'pelvis force (up+/down)';
subplotTitle{2}  = 'pelvis force (fore+/aft)';
subplotTitle{3}  = 'pelvis force (R+/L)';
subplotTitle{4}  = 'back bending moment (R down+/up)';
subplotTitle{5}  = 'back extension moment (post+/ant)';
subplotTitle{6}  = 'back rotation moment (R int+/ext)';
subplotTitle{7}  = 'pelv obliquity moment (R down+/up)';
subplotTitle{8}  = 'pelv tilt moment (post+/ant)';
subplotTitle{9}  = 'pelv rotation moment (R int+/ext)';
subplotTitle{10} = {};
subplotTitle{11} = {};
subplotTitle{12} = {};
subplotAxisLabel{1}  = '(N)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = '(Nm)';
subplotAxisLabel{5}  = {};
subplotAxisLabel{6}  = {};
subplotAxisLabel{7}  = '(Nm)';
subplotAxisLabel{8}  = {};
subplotAxisLabel{9}  = {};
subplotAxisLabel{10} = {};
subplotAxisLabel{11} = {};
subplotAxisLabel{12} = {};
subplotRange{1}  = [-250 250];
subplotRange{2}  = [-250 250];
subplotRange{3}  = [-250 250];
subplotRange{4}  = [-50 50];
subplotRange{5}  = [-50 50];
subplotRange{6}  = [-50 50];
subplotRange{7}  = [-50 50];
subplotRange{8}  = [-50 50];
subplotRange{9}  = [-50 50];
subplotRange{10} = {};
subplotRange{11} = {};
subplotRange{12} = {};
subplotTick{1}  = [-200:200:200];
subplotTick{2}  = [-200:200:200];
subplotTick{3}  = [-200:200:200];
subplotTick{4}  = [-40:40:40];
subplotTick{5}  = [-40:40:40];
subplotTick{6}  = [-40:40:40];
subplotTick{7}  = [-40:40:40];
subplotTick{8}  = [-40:40:40];
subplotTick{9}  = [-40:40:40];
subplotTick{10} = {};
subplotTick{11} = {};
subplotTick{12} = {};
figContents(1).qPlotLabel = qPlotLabel;
figContents(1).subplotTitle = subplotTitle;
figContents(1).subplotAxisLabel = subplotAxisLabel;
figContents(1).subplotRange = subplotRange;
figContents(1).subplotTick = subplotTick;
clear qPlotLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;


% Figure 2:  R Hip, Knee, and Ankle Moments
qPlotLabel{1}  = 'hip_adduction_r_torque';
qPlotLabel{2}  = 'hip_flexion_r_torque';
qPlotLabel{3}  = 'hip_rotation_r_torque';
qPlotLabel{4}  = {};
qPlotLabel{5}  = 'knee_angle_r_torque';
qPlotLabel{6}  = {};
qPlotLabel{7}  = 'subtalar_angle_r_torque';
qPlotLabel{8}  = 'ankle_angle_r_torque';
qPlotLabel{9}  = 'mtp_angle_r_torque';
qPlotLabel{10} = {};
qPlotLabel{11} = {};
qPlotLabel{12} = {};
subplotTitle{1}  = 'R hip ad+/abduction moment';
subplotTitle{2}  = 'R hip flex+/extension moment';
subplotTitle{3}  = 'R hip int+/ext rotation moment';
subplotTitle{4}  = {};
subplotTitle{5}  = 'R knee flex/extension+ moment';
subplotTitle{6}  = {};
subplotTitle{7}  = 'R subtalar inv+/eversion moment';
subplotTitle{8}  = 'R ankle df+/pf moment';
subplotTitle{9}  = 'R mtp flex+/extension moment';
subplotTitle{10} = {};
subplotTitle{11} = {};
subplotTitle{12} = {};
subplotAxisLabel{1}  = '(Nm)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = {};
subplotAxisLabel{5}  = '(Nm)';
subplotAxisLabel{6}  = {};
subplotAxisLabel{7}  = '(Nm)';
subplotAxisLabel{8}  = {};
subplotAxisLabel{9}  = {};
subplotAxisLabel{10} = {};
subplotAxisLabel{11} = {};
subplotAxisLabel{12} = {};
subplotRange{1}  = [-70 70];
subplotRange{2}  = [-70 70];
subplotRange{3}  = [-40 40];
subplotRange{4}  = {};
subplotRange{5}  = [-40 40];
subplotRange{6}  = {};
subplotRange{7}  = [-25 25];
subplotRange{8}  = [-100 20];
subplotRange{9}  = [-25 25];
subplotRange{10} = {};
subplotRange{11} = {};
subplotRange{12} = {};
subplotTick{1}  = [-40:40:40];
subplotTick{2}  = [-40:40:40];
subplotTick{3}  = [-40:40:40];
subplotTick{4}  = {};
subplotTick{5}  = [-40:40:40];
subplotTick{6}  = {};
subplotTick{7}  = [-20:20:20];
subplotTick{8}  = [-80:40:20];
subplotTick{9}  = [-20:20:20];
subplotTick{10} = {};
subplotTick{11} = {};
subplotTick{12} = {};
figContents(2).qPlotLabel = qPlotLabel;
figContents(2).subplotTitle = subplotTitle;
figContents(2).subplotAxisLabel = subplotAxisLabel;
figContents(2).subplotRange = subplotRange;
figContents(2).subplotTick = subplotTick;
clear qPlotLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;


% Figure 3:  L Hip, Knee, and Ankle Moments
qPlotLabel{1}  = 'hip_adduction_l_torque';
qPlotLabel{2}  = 'hip_flexion_l_torque';
qPlotLabel{3}  = 'hip_rotation_l_torque';
qPlotLabel{4}  = {};
qPlotLabel{5}  = 'knee_angle_l_torque';
qPlotLabel{6}  = {};
qPlotLabel{7}  = 'subtalar_angle_l_torque';
qPlotLabel{8}  = 'ankle_angle_l_torque';
qPlotLabel{9}  = 'mtp_angle_l_torque';
qPlotLabel{10} = {};
qPlotLabel{11} = {};
qPlotLabel{12} = {};
subplotTitle{1}  = 'L hip ad+/abduction moment';
subplotTitle{2}  = 'L hip flex+/extension moment';
subplotTitle{3}  = 'L hip int+/ext rotation moment';
subplotTitle{4}  = {};
subplotTitle{5}  = 'L knee flex/extension+ moment';
subplotTitle{6}  = {};
subplotTitle{7}  = 'L subtalar inv+/eversion moment';
subplotTitle{8}  = 'L ankle df+/pf moment';
subplotTitle{9}  = 'L mtp flex+/extension moment';
subplotTitle{10} = {};
subplotTitle{11} = {};
subplotTitle{12} = {};
subplotAxisLabel{1}  = '(Nm)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = {};
subplotAxisLabel{5}  = '(Nm)';
subplotAxisLabel{6}  = {};
subplotAxisLabel{7}  = '(Nm)';
subplotAxisLabel{8}  = {};
subplotAxisLabel{9}  = {};
subplotAxisLabel{10} = {};
subplotAxisLabel{11} = {};
subplotAxisLabel{12} = {};
subplotRange{1}  = [-70 70];
subplotRange{2}  = [-70 70];
subplotRange{3}  = [-40 40];
subplotRange{4}  = {};
subplotRange{5}  = [-40 40];
subplotRange{6}  = {};
subplotRange{7}  = [-25 25];
subplotRange{8}  = [-100 20];
subplotRange{9}  = [-25 25];
subplotRange{10} = {};
subplotRange{11} = {};
subplotRange{12} = {};
subplotTick{1}  = [-40:40:40];
subplotTick{2}  = [-40:40:40];
subplotTick{3}  = [-40:40:40];
subplotTick{4}  = {};
subplotTick{5}  = [-40:40:40];
subplotTick{6}  = {};
subplotTick{7}  = [-20:20:20];
subplotTick{8}  = [-80:40:20];
subplotTick{9}  = [-20:20:20];
subplotTick{10} = {};
subplotTick{11} = {};
subplotTick{12} = {};
figContents(3).qPlotLabel = qPlotLabel;
figContents(3).subplotTitle = subplotTitle;
figContents(3).subplotAxisLabel = subplotAxisLabel;
figContents(3).subplotRange = subplotRange;
figContents(3).subplotTick = subplotTick;
clear qPlotLabel; 
clear subplotTitle subplotAxisLabel subplotRange subplotTick;

return;