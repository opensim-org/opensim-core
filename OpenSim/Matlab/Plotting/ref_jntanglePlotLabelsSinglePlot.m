function figContents = ref_jntanglePlotLabelsSinglePlot()
% Purpose:  Returns a list of labels specifying the data to be plotted
%           in each subplot of each figure created by 
%           compare_jntanglesFromMotSinglePlot(). 
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


% Figs. 1-9: Pelvis Translations, Back Angles, and Pelvis Angles
figContents(1).qPlotLabel{1}  = 'pelvis_ty';
figContents(2).qPlotLabel{1}  = 'pelvis_tx';
figContents(3).qPlotLabel{1}  = 'pelvis_tz';
figContents(4).qPlotLabel{1}  = 'lumbar_bending';
figContents(5).qPlotLabel{1}  = 'lumbar_extension';
figContents(6).qPlotLabel{1}  = 'lumbar_rotation';
figContents(7).qPlotLabel{1}  = 'pelvis_list';
figContents(8).qPlotLabel{1}  = 'pelvis_tilt';
figContents(9).qPlotLabel{1}  = 'pelvis_rotation';
figContents(1).qMeasuredLabel{1}  = 'pelvis_ty';
figContents(2).qMeasuredLabel{1}  = 'pelvis_tx';
figContents(3).qMeasuredLabel{1}  = 'pelvis_tz';
figContents(4).qMeasuredLabel{1}  = 'lumbar_bending';
figContents(5).qMeasuredLabel{1}  = 'lumbar_extension';
figContents(6).qMeasuredLabel{1}  = 'lumbar_rotation';
figContents(7).qMeasuredLabel{1}  = 'pelvis_list';
figContents(8).qMeasuredLabel{1}  = 'pelvis_tilt';
figContents(9).qMeasuredLabel{1}  = 'pelvis_rotation';
figContents(1).gcdMeanLabel{1}  = {};
figContents(2).gcdMeanLabel{1}  = {};
figContents(3).gcdMeanLabel{1}  = {};
figContents(4).gcdMeanLabel{1}  = {};
figContents(5).gcdMeanLabel{1}  = {};
figContents(6).gcdMeanLabel{1}  = {};
figContents(7).gcdMeanLabel{1}  = 'PelvicObliquity';
figContents(8).gcdMeanLabel{1}  = 'PelvicTilt';
figContents(9).gcdMeanLabel{1}  = 'PelvicRotation';
figContents(1).subplotTitle{1}  = 'pelvis translation (up+/down)';
figContents(2).subplotTitle{1}  = 'pelvis translation (fore+/aft)';
figContents(3).subplotTitle{1}  = 'pelvis translation (R+/L)';
figContents(4).subplotTitle{1}  = 'back bending (R down+/up)';
figContents(5).subplotTitle{1}  = 'back extension (post+/ant)';
figContents(6).subplotTitle{1}  = 'back rotation (R int+/ext)';
figContents(7).subplotTitle{1}  = 'pelvic obliquity (R up+/down)';
figContents(8).subplotTitle{1}  = 'pelvic tilt (ant+/post)';
figContents(9).subplotTitle{1}  = 'pelvic rotation (R int+/ext)';
figContents(1).subplotAxisLabel{1}  = 'translation (m)';
figContents(2).subplotAxisLabel{1}  = 'translation (m)';
figContents(3).subplotAxisLabel{1}  = 'translation (m)';
figContents(4).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(5).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(6).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(7).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(8).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(9).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(1).subplotRange{1}  = {};
figContents(2).subplotRange{1}  = {};
figContents(3).subplotRange{1}  = {};
figContents(4).subplotRange{1}  = [-10 15];
figContents(5).subplotRange{1}  = [-25 -5];
figContents(6).subplotRange{1}  = [-20 20];
figContents(7).subplotRange{1}  = [-10 10];
figContents(8).subplotRange{1}  = [0 20];
figContents(9).subplotRange{1}  = [-20 10];
figContents(1).subplotTick{1}  = {};
figContents(2).subplotTick{1}  = {};
figContents(3).subplotTick{1}  = {};
figContents(4).subplotTick{1}  = -10:5:15;
figContents(5).subplotTick{1}  = -25:5:-5;
figContents(6).subplotTick{1}  = -20:10:20;
figContents(7).subplotTick{1}  = -10:5:10;
figContents(8).subplotTick{1}  = 0:5:20;
figContents(9).subplotTick{1}  = -20:10:10;


% Figs. 10-16: R Hip, Knee, and Ankle Angles
figContents(10).qPlotLabel{1}  = 'hip_adduction_r';
figContents(11).qPlotLabel{1}  = 'hip_flexion_r';
figContents(12).qPlotLabel{1}  = 'hip_rotation_r';
figContents(13).qPlotLabel{1}  = 'knee_angle_r';
figContents(14).qPlotLabel{1}  = 'subtalar_angle_r';
figContents(15).qPlotLabel{1}  = 'ankle_angle_r';
figContents(16).qPlotLabel{1}  = 'mtp_angle_r';
figContents(10).qMeasuredLabel{1}  = 'hip_adduction_r';
figContents(11).qMeasuredLabel{1}  = 'hip_flexion_r';
figContents(12).qMeasuredLabel{1}  = 'hip_rotation_r';
figContents(13).qMeasuredLabel{1}  = 'knee_angle_r';
figContents(14).qMeasuredLabel{1}  = 'subtalar_angle_r';
figContents(15).qMeasuredLabel{1}  = 'ankle_angle_r';
figContents(16).qMeasuredLabel{1}  = 'mtp_angle_r';
figContents(10).gcdMeanLabel{1}  = 'HipAbAdduct';
figContents(11).gcdMeanLabel{1}  = 'HipFlexExt';
figContents(12).gcdMeanLabel{1}  = 'HipRotation';
figContents(13).gcdMeanLabel{1}  = 'KneeFlexExt';
figContents(14).gcdMeanLabel{1}  = {};
figContents(15).gcdMeanLabel{1}  = 'DorsiPlanFlex';
figContents(16).gcdMeanLabel{1}  = {};
figContents(10).subplotTitle{1}  = 'R hip ad+/abduction';
figContents(11).subplotTitle{1}  = 'R hip flex+/extension';
figContents(12).subplotTitle{1}  = 'R hip int+/ext rotation';
figContents(13).subplotTitle{1}  = 'R knee flex+/extension';
figContents(14).subplotTitle{1}  = 'R subtalar inv+/eversion';
figContents(15).subplotTitle{1}  = 'R ankle dorsi+/plantarflexion';
figContents(16).subplotTitle{1}  = 'R mtp flex+/extension';
figContents(10).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(11).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(12).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(13).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(14).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(15).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(16).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(10).subplotRange{1} = [-10 10];
figContents(11).subplotRange{1} = [-20 60];
figContents(12).subplotRange{1}  = [-20 10];
figContents(13).subplotRange{1} = [-5 85];
figContents(14).subplotRange{1}  = [-20 20];
figContents(15).subplotRange{1} = [-20 30];
figContents(16).subplotRange{1}  = [-20 20];
figContents(10).subplotTick{1} = -15:5:10;
figContents(11).subplotTick{1} = -40:20:60;
figContents(12).subplotTick{1}  = -30:20:30;
figContents(13).subplotTick{1} = 0:20:80;
figContents(14).subplotTick{1}  = -20:20:20;
figContents(15).subplotTick{1} = -20:10:30;
figContents(16).subplotTick{1}  = -20:20:20;


% Figs. 17-23:  L Hip, Knee, and Ankle Angles
figContents(17).qPlotLabel{1}  = 'hip_adduction_l';
figContents(18).qPlotLabel{1}  = 'hip_flexion_l';
figContents(19).qPlotLabel{1}  = 'hip_rotation_l';
figContents(20).qPlotLabel{1}  = 'knee_angle_l';
figContents(21).qPlotLabel{1}  = 'subtalar_angle_l';
figContents(22).qPlotLabel{1}  = 'ankle_angle_l';
figContents(23).qPlotLabel{1}  = 'mtp_angle_l';
figContents(17).qMeasuredLabel{1}  = 'hip_adduction_l';
figContents(18).qMeasuredLabel{1}  = 'hip_flexion_l';
figContents(19).qMeasuredLabel{1}  = 'hip_rotation_l';
figContents(20).qMeasuredLabel{1}  = 'knee_angle_l';
figContents(21).qMeasuredLabel{1}  = 'subtalar_angle_l';
figContents(22).qMeasuredLabel{1}  = 'ankle_angle_l';
figContents(23).qMeasuredLabel{1}  = 'mtp_angle_l';
figContents(17).gcdMeanLabel{1}  = 'HipAbAdduct';
figContents(18).gcdMeanLabel{1}  = 'HipFlexExt';
figContents(19).gcdMeanLabel{1}  = 'HipRotation';
figContents(20).gcdMeanLabel{1}  = 'KneeFlexExt';
figContents(21).gcdMeanLabel{1}  = {};
figContents(22).gcdMeanLabel{1}  = 'DorsiPlanFlex';
figContents(23).gcdMeanLabel{1}  = {};
figContents(17).subplotTitle{1}  = 'L hip ad+/abduction';
figContents(18).subplotTitle{1}  = 'L hip flex+/extension';
figContents(19).subplotTitle{1}  = 'L hip int+/ext rotation';
figContents(20).subplotTitle{1}  = 'L knee flex+/extension';
figContents(21).subplotTitle{1}  = 'L subtalar inv+/eversion';
figContents(22).subplotTitle{1}  = 'L ankle dorsi+/plantarflexion';
figContents(23).subplotTitle{1}  = 'L mtp flex+/extension';
figContents(17).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(18).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(19).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(20).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(21).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(22).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(23).subplotAxisLabel{1}  = 'joint angle (deg)';
figContents(17).subplotRange{1} = [-10 10];
figContents(18).subplotRange{1} = [-20 60];
figContents(19).subplotRange{1} = [-20 10];
figContents(20).subplotRange{1} = [-5 85];
figContents(21).subplotRange{1} = [-20 20];
figContents(22).subplotRange{1} = [-20 30];
figContents(23).subplotRange{1} = [-20 20];
figContents(17).subplotTick{1} = -15:5:10;
figContents(18).subplotTick{1} = -40:20:60;
figContents(19).subplotTick{1} = -20:10:10;
figContents(20).subplotTick{1} = 0:20:80;
figContents(21).subplotTick{1} = -20:20:20;
figContents(22).subplotTick{1} = -20:10:30;
figContents(23).subplotTick{1} = -20:20:20;

return;
