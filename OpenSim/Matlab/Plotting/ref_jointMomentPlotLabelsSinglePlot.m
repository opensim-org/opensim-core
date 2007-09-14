function figContents = ref_jointMomentPlotLabelsSinglePlot()
% Purpose:  Returns a list of labels specifying the data to be plotted
%           in each subplot of each figure created by 
%           compare_jointMomentsFromMotSinglePlot(). 
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


% Figs. 1-6: Pelvis Forces and Moments
figContents(1).qPlotLabel{1}  = {'pelvis_tx_ik_moment', 'pelvis_tx_cmc_moment'};
figContents(2).qPlotLabel{1}  = {'pelvis_list_ik_moment', 'pelvis_list_cmc_moment'};;
figContents(3).qPlotLabel{1}  = {'pelvis_ty_ik_moment', 'pelvis_ty_cmc_moment'};;
figContents(4).qPlotLabel{1}  = {'pelvis_rotation_ik_moment', 'pelvis_rotation_cmc_moment'};
figContents(5).qPlotLabel{1}  = {'pelvis_tz_ik_moment', 'pelvis_tz_cmc_moment'};
figContents(6).qPlotLabel{1}  = {'pelvis_tilt_ik_moment', 'pelvis_tilt_cmc_moment'};
figContents(1).subplotTitle{1}  = {'Pelvis force (fore+/aft)'};
figContents(2).subplotTitle{1}  = {'Pelv obliquity moment (R down+/up)'};
figContents(3).subplotTitle{1}  = {'Pelvis force (up+/down)'};
figContents(4).subplotTitle{1}  = {'Pelv rotation moment (R int+/ext)'};
figContents(5).subplotTitle{1}  = {'Pelvis force (R+/L)'};
figContents(6).subplotTitle{1}  = {'Pelv tilt moment (post+/ant)'};
figContents(1).subplotAxisLabel{1}  = 'force (N)';
figContents(2).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(3).subplotAxisLabel{1}  = 'force (N)';
figContents(4).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(5).subplotAxisLabel{1}  = 'force (N)';
figContents(6).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(1).subplotRange{1}  = {};
figContents(2).subplotRange{1}  = {};
figContents(3).subplotRange{1}  = {};
figContents(4).subplotRange{1}  = {};
figContents(5).subplotRange{1}  = {};
figContents(6).subplotRange{1}  = {};
figContents(1).subplotTick{1}  = {};
figContents(2).subplotTick{1}  = {};
figContents(3).subplotTick{1}  = {};
figContents(4).subplotTick{1}  = {};
figContents(5).subplotTick{1}  = {};
figContents(6).subplotTick{1}  = {};


% Figs. 7-12: Hip Moments
figContents(7).qPlotLabel{1}  = {'hip_flexion_l_ik_moment', 'hip_flexion_l_cmc_moment'};
figContents(8).qPlotLabel{1}  = {'hip_flexion_r_ik_moment', 'hip_flexion_r_cmc_moment'};
figContents(9).qPlotLabel{1}  = {'hip_adduction_l_ik_moment', 'hip_adduction_l_cmc_moment'};
figContents(10).qPlotLabel{1}  = {'hip_adduction_r_ik_moment', 'hip_adduction_r_cmc_moment'};
figContents(11).qPlotLabel{1}  = {'hip_rotation_l_ik_moment', 'hip_rotation_l_cmc_moment'};
figContents(12).qPlotLabel{1}  = {'hip_rotation_r_ik_moment', 'hip_rotation_r_cmc_moment'};
figContents(7).subplotTitle{1}  = {'L hip flex+/extension moment'};
figContents(8).subplotTitle{1}  = {'R hip flex+/extension moment'};
figContents(9).subplotTitle{1}  = {'L hip ad+/abduction moment'};
figContents(10).subplotTitle{1}  = {'R hip ad+/abduction moment'};
figContents(11).subplotTitle{1}  = {'L hip int+/ext rotation moment'};
figContents(12).subplotTitle{1}  = {'R hip int+/ext rotation moment'};
figContents(7).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(8).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(9).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(10).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(11).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(12).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(7).subplotRange{1} = {};
figContents(8).subplotRange{1} = {};
figContents(9).subplotRange{1}  = {};
figContents(10).subplotRange{1} = {};
figContents(11).subplotRange{1}  = {};
figContents(12).subplotRange{1} = {};
figContents(7).subplotTick{1} = {};
figContents(8).subplotTick{1} = {};
figContents(9).subplotTick{1}  = {};
figContents(10).subplotTick{1} = {};
figContents(11).subplotTick{1}  = {};
figContents(12).subplotTick{1} = {};


% Figs. 13-16:  Knee and Ankle Moments
figContents(13).qPlotLabel{1}  = {'knee_angle_l_ik_moment', 'knee_angle_l_cmc_moment'};
figContents(14).qPlotLabel{1}  = {'knee_angle_r_ik_moment', 'knee_angle_r_cmc_moment'};
figContents(15).qPlotLabel{1}  = {'ankle_angle_l_ik_moment', 'ankle_angle_l_cmc_moment'};
figContents(16).qPlotLabel{1}  = {'ankle_angle_r_ik_moment', 'ankle_angle_r_cmc_moment'};
figContents(13).subplotTitle{1}  = {'L knee flex/extension+ moment'};
figContents(14).subplotTitle{1}  = {'R knee flex/extension+ moment'};
figContents(15).subplotTitle{1}  = {'L ankle df+/pf moment'};
figContents(16).subplotTitle{1}  = {'R ankle df+/pf moment'};
figContents(13).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(14).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(15).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(16).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(13).subplotRange{1} = [-40 20];
figContents(14).subplotRange{1} = [-40 20];
figContents(15).subplotRange{1} = {};
figContents(16).subplotRange{1} = {};
figContents(13).subplotTick{1} = -40:20:20;
figContents(14).subplotTick{1} = -40:20:20;
figContents(15).subplotTick{1} = {};
figContents(16).subplotTick{1} = {};


% Figs. 17-19: Low Back Joint Moments
figContents(17).qPlotLabel{1}  = {'lumbar_extension_ik_moment', 'lumbar_extension_cmc_moment'};
figContents(18).qPlotLabel{1}  = {'lumbar_bending_ik_moment', 'lumbar_bending_cmc_moment'};
figContents(19).qPlotLabel{1}  = {'lumbar_rotation_ik_moment', 'lumbar_rotation_cmc_moment'};
figContents(17).subplotTitle{1}  = {'Back extension moment (post+/ant)'};
figContents(18).subplotTitle{1}  = {'Back bending moment (R down+/up)'};
figContents(19).subplotTitle{1}  = {'Back rotation moment (R int+/ext)'};
figContents(17).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(18).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(19).subplotAxisLabel{1}  = 'moment (N \cdot m)';
figContents(17).subplotRange{1} = {};
figContents(18).subplotRange{1} = {};
figContents(19).subplotRange{1} = {};
figContents(17).subplotTick{1} = {};
figContents(18).subplotTick{1} = {};
figContents(19).subplotTick{1} = {};

return;
