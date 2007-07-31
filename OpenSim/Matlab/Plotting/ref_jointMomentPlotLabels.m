function figContents = ref_jointMomentPlotLabels()
% Purpose:  Returns a list of labels specifying the data to be plotted
%           in each subplot of each figure created by 
%           compare_jointMomFromMot(). 
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
% CTJ, 07-07, adapted from:
% ASA, 11-05, rev 2-06


% Figure 1: Pelvis-ground joint moments.
qPlotLabel{1}  = {'ground_force_vy'};
qPlotLabel{2}  = {'ground_force_vy'};
qPlotLabel{3}  = {};
qPlotLabel{4}  = {};
qPlotLabel{5}  = {'pelvis_tx_moment'};
qPlotLabel{6}  = {'pelvis_list_moment'};
qPlotLabel{7}  = {'pelvis_ty_moment'};
qPlotLabel{8}  = {'pelvis_rotation_moment'};
qPlotLabel{9}  = {'pelvis_tz_moment'};
qPlotLabel{10} = {'pelvis_tilt_moment'};
subplotTitle{1}  = {'Left'};
subplotTitle{2}  = {'Right'};
subplotTitle{3}  = {};
subplotTitle{4}  = {};
subplotTitle{5}  = {'Pelvis force (fore+/aft)'};
subplotTitle{6}  = {'Pelv obliquity moment (R down+/up)'};
subplotTitle{7}  = {'Pelvis force (up+/down)'};
subplotTitle{8}  = {'Pelv rotation moment (R int+/ext)'};
subplotTitle{9}  = {'Pelvis force (R+/L)'};
subplotTitle{10} = {'Pelv tilt moment (post+/ant)'};
subplotAxisLabel{1}  = 'vertical GRF (N)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = {};
subplotAxisLabel{5}  = 'force (N)';
subplotAxisLabel{6}  = 'moment (N \cdot m)';
subplotAxisLabel{7}  = 'force (N)';
subplotAxisLabel{8}  = 'moment (N \cdot m)';
subplotAxisLabel{9}  = 'force (N)';
subplotAxisLabel{10} = 'moment (N \cdot m)';
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


% Figure 2: Hip moments.
qPlotLabel{1}  = {'ground_force_vy'};
qPlotLabel{2}  = {'ground_force_vy'};
qPlotLabel{3}  = {};
qPlotLabel{4}  = {};
qPlotLabel{5}  = {'hip_flexion_l_moment'};
qPlotLabel{6}  = {'hip_flexion_r_moment'};
qPlotLabel{7}  = {'hip_adduction_l_moment'};
qPlotLabel{8}  = {'hip_adduction_r_moment'};
qPlotLabel{9}  = {'hip_rotation_l_moment'};
qPlotLabel{10} = {'hip_rotation_r_moment'};
subplotTitle{1}  = {'Left'};
subplotTitle{2}  = {'Right'};
subplotTitle{3}  = {};
subplotTitle{4}  = {};
subplotTitle{5}  = {'L hip flex+/extension moment'};
subplotTitle{6}  = {'R hip flex+/extension moment'};
subplotTitle{7}  = {'L hip ad+/abduction moment'};
subplotTitle{8}  = {'R hip ad+/abduction moment'};
subplotTitle{9}  = {'L hip int+/ext rotation moment'};
subplotTitle{10} = {'R hip int+/ext rotation moment'};
subplotAxisLabel{1}  = 'vertical GRF (N)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = {};
subplotAxisLabel{5}  = 'moment (N \cdot m)';
subplotAxisLabel{6}  = {};
subplotAxisLabel{7}  = 'moment (N \cdot m)';
subplotAxisLabel{8}  = {};
subplotAxisLabel{9}  = 'moment (N \cdot m)';
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


% Figure 3: Knee and ankle flexion moments.
qPlotLabel{1}  = {'ground_force_vy'};
qPlotLabel{2}  = {'ground_force_vy'};
qPlotLabel{3}  = {};
qPlotLabel{4}  = {};
qPlotLabel{5}  = {'knee_angle_l_moment'};
qPlotLabel{6}  = {'knee_angle_r_moment'};
qPlotLabel{7}  = {'ankle_angle_l_moment'};
qPlotLabel{8}  = {'ankle_angle_r_moment'};
qPlotLabel{9}  = {};
qPlotLabel{10} = {};
subplotTitle{1}  = {'Left'};
subplotTitle{2}  = {'Right'};
subplotTitle{3}  = {};
subplotTitle{4}  = {};
subplotTitle{5}  = {'L knee flex/extension+ moment'};
subplotTitle{6}  = {'R knee flex/extension+ moment'};
subplotTitle{7}  = {'L ankle df+/pf moment'};
subplotTitle{8}  = {'R ankle df+/pf moment'};
subplotTitle{9}  = {};
subplotTitle{10} = {};
subplotAxisLabel{1}  = 'vertical GRF (N)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = {};
subplotAxisLabel{5}  = 'moment (N \cdot m)';
subplotAxisLabel{6}  = {};
subplotAxisLabel{7}  = 'moment (N \cdot m)';
subplotAxisLabel{8}  = {};
subplotAxisLabel{9}  = 'moment (N \cdot m)';
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


% Figure 4: Low back joint moments.
qPlotLabel{1}  = {'ground_force_vy'};
qPlotLabel{2}  = {'ground_force_vy'};
qPlotLabel{3}  = {};
qPlotLabel{4}  = {};
qPlotLabel{5}  = {'lumbar_extension_moment'};
qPlotLabel{6}  = {};
qPlotLabel{7}  = {'lumbar_bending_moment'};
qPlotLabel{8}  = {};
qPlotLabel{9}  = {'lumbar_rotation_moment'};
qPlotLabel{10} = {};
subplotTitle{1}  = {'Left'};
subplotTitle{2}  = {'Right'};
subplotTitle{3}  = {};
subplotTitle{4}  = {};
subplotTitle{5}  = {'Back extension moment (post+/ant)'};
subplotTitle{6}  = {};
subplotTitle{7}  = {'Back bending moment (R down+/up)'};
subplotTitle{8}  = {};
subplotTitle{9}  = {'Back rotation moment (R int+/ext)'};
subplotTitle{10} = {};
subplotAxisLabel{1}  = 'vertical GRF (N)';
subplotAxisLabel{2}  = {};
subplotAxisLabel{3}  = {};
subplotAxisLabel{4}  = {};
subplotAxisLabel{5}  = 'moment (N \cdot m)';
subplotAxisLabel{6}  = {};
subplotAxisLabel{7}  = 'moment (N \cdot m)';
subplotAxisLabel{8}  = {};
subplotAxisLabel{9}  = 'moment (N \cdot m)';
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
