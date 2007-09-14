function script_compareDataFromMotSinglePlot()

%
% Add comment here describing all parameters, and what function does!
%

FAST = 1;
FREE = 2;
SLOW = 3;

subjectnumber = '2';
%subjectnumber = '3';
subjectname = ['de' num2str(subjectnumber)];
trialnames = {'fast_walk1' 'ss_walk1' 'vslow_walk1'};
timeAxisLabel = 'percent of gait cycle';
%timeAxisLabel = 'time (s)';

%if nargin < 3
%	timeAxisLabel = 'time (s)';
%end
if nargin < 4
    bgColor = 'w';
end
if nargin < 5
    fgColor = 'k';
end
if nargin < 6
    gcLimb = 'L';
end
if ~strcmp(gcLimb, 'L') && ~strcmp(gcLimb, 'R')
    error( '4th parameter (gcLimb) should be either L or R.' );
end
if nargin < 7
	individualprintmenus = 0;
end

% Other "inputs" (make these arguments?)
%desiredJointAngles = {'lumbar_bending' 'lumbar_extension' ...
%    'lumbar_rotation' 'pelvis_list' 'pelvis_tilt' 'pelvis_rotation' ...
%    'hip_adduction_l' 'hip_flexion_l' 'hip_rotation_l' 'knee_angle_l' ...
%    'ankle_angle_l'};
desiredJointAngles = {};
desiredJointMoments = {'lumbar_bending' 'lumbar_extension' ...
    'lumbar_rotation' 'pelvis_list' 'pelvis_tilt' 'pelvis_rotation' ...
    'hip_adduction_l' 'hip_flexion_l' 'hip_rotation_l' 'knee_angle_l' ...
    'ankle_angle_l'};
%desiredJointMoments = {};
%desiredMuscleActivations = { 'glut_max_l' 'glut_med_l' ...
%    'glut_min_l' 'iliacus_l' 'sar_l' ...
%    'tfl_l' 'vas_l' 'rect_fem_l' 'semimem_l' ...
%    'bifemlh_l' 'bifemsh_l' 'add_brev_l' ...
%    'add_mag_l' 'gas_l' 'soleus_l' ...
%    'flex_dig_l' 'tib_ant_l' 'ext_dig_l' ...
%    'ercspn_l' 'intobl_l' ...
%    'extobl_l' };
desiredMuscleActivations = {};

global GLOBAL_individualprintmenus GLOBAL_figHandles;
GLOBAL_figHandles = [];
GLOBAL_individualprintmenus = individualprintmenus;

cd (['D:\programfiles\FCA\SU\Testing\delaware' num2str(subjectnumber) '_1.5_fast_auto']);
[sInfo, fastInfo] = ref_trialInfoDelaware2(subjectname, trialnames{FAST});
cd (['D:\programfiles\FCA\SU\Testing\delaware' num2str(subjectnumber) '_1.5_ss_auto']);
[sInfo, freeInfo] = ref_trialInfoDelaware2(subjectname, trialnames{FREE});
cd (['D:\programfiles\FCA\SU\Testing\delaware' num2str(subjectnumber) '_1.5_vslow_auto']);
[sInfo, slowInfo] = ref_trialInfoDelaware2(subjectname, trialnames{SLOW});
fastTrial = fastInfo.(trialnames{FAST});
freeTrial = freeInfo.(trialnames{FREE});
slowTrial = slowInfo.(trialnames{SLOW});
ref_dataFormat = ref_dataFormatDelaware;

subject = sInfo.subject;
fastInfo.mass = sInfo.mass;
freeInfo.mass = sInfo.mass;
slowInfo.mass = sInfo.mass;
fastInfo.analogRate = ref_dataFormat.analogRate;
freeInfo.analogRate = ref_dataFormat.analogRate;
slowInfo.analogRate = ref_dataFormat.analogRate;

eval(sprintf('fastTrial = fastInfo.%s;', trialnames{FAST}));
eval(sprintf('freeTrial = freeInfo.%s;', trialnames{FREE}));
eval(sprintf('slowTrial = slowInfo.%s;', trialnames{SLOW}));

fastInfo.trial = fastTrial.trial;
fastInfo.speed = fastTrial.speed;
fastInfo.ictoMatrix = fastTrial.ictoMatrix;
fastInfo.FP = fastTrial.FP;
fastInfo.limb = fastTrial.limb;
fastInfo.timeAxisLabel = timeAxisLabel;
fastInfo.gcLimb = gcLimb;
fastInfo.bgColor = bgColor;
fastInfo.fgColor = fgColor;

freeInfo.trial = freeTrial.trial;
freeInfo.speed = freeTrial.speed;
freeInfo.ictoMatrix = freeTrial.ictoMatrix;
freeInfo.FP = freeTrial.FP;
freeInfo.limb = freeTrial.limb;
freeInfo.timeAxisLabel = timeAxisLabel;
freeInfo.gcLimb = gcLimb;
freeInfo.bgColor = bgColor;
freeInfo.fgColor = fgColor;

slowInfo.trial = slowTrial.trial;
slowInfo.speed = slowTrial.speed;
slowInfo.ictoMatrix = slowTrial.ictoMatrix;
slowInfo.FP = slowTrial.FP;
slowInfo.limb = slowTrial.limb;
slowInfo.timeAxisLabel = timeAxisLabel;
slowInfo.gcLimb = gcLimb;
slowInfo.bgColor = bgColor;
slowInfo.fgColor = fgColor;

datadir = 'D:\programfiles\FCA\SU\Testing';
if strcmp(subjectname, 'de2')
    de2dirs = { fullfile( datadir, 'delaware2_1.5_fast_auto' ), ...
                fullfile( datadir, 'delaware2_1.5_ss_auto' ), ...
                fullfile( datadir, 'delaware2_1.5_vslow_auto' ) };
elseif strcmp(subjectname, 'de3')
    de2dirs = { fullfile( datadir, 'delaware3_1.5_fast_auto' ), ...
                fullfile( datadir, 'delaware3_1.5_ss_auto' ), ...
                fullfile( datadir, 'delaware3_1.5_vslow_auto' ) };
else
    error( ['Subjectname ' subjectname 'does not exist!'] );
end

%
% Joint angles
%

% Get figure numbers and plot properties
figContentsAll = ref_jntanglePlotLabelsSinglePlot;
allJointAngles = {'pelvis_ty' 'pelvis_tx' 'pelvis_tz' 'lumbar_bending' 'lumbar_extension' ...
    'lumbar_rotation' 'pelvis_list' 'pelvis_tilt' 'pelvis_rotation' 'hip_adduction_r' ...
    'hip_flexion_r' 'hip_rotation_r' 'knee_angle_r' 'subtalar_angle_r' 'ankle_angle_r' ...
    'mtp_angle_r' 'hip_adduction_l' 'hip_flexion_l' 'hip_rotation_l' 'knee_angle_l' ...
    'subtalar_angle_l' 'ankle_angle_l' 'mtp_angle_l'};
figIndices = [];
for i = 1:length( desiredJointAngles )
    indicesOfCurrentJointAngle = find( strcmp( desiredJointAngles{i}, allJointAngles ) );
    figIndices = [figIndices indicesOfCurrentJointAngle];
end
figContents = figContentsAll( figIndices );

fnames = { fullfile( de2dirs{FAST}, [subject '_' fastInfo.trial '_ik.mot' ] )
           fullfile( de2dirs{FAST}, ['ResultsCMC/' subject '_' fastInfo.trial '_Kinematics_q.mot'] )
           fullfile( de2dirs{FREE}, [subject '_' freeInfo.trial '_ik.mot' ] )
           fullfile( de2dirs{FREE}, ['ResultsCMC/' subject '_' freeInfo.trial '_Kinematics_q.mot'] )
           fullfile( de2dirs{SLOW}, [subject '_' slowInfo.trial '_ik.mot' ] )
           fullfile( de2dirs{SLOW}, ['ResultsCMC/' subject '_' slowInfo.trial '_Kinematics_q.mot'] ) };
fnameMeasured = {};
figHandleArray = 1:length( figIndices );
currentFigNumberLimit = length( figIndices );
tInfo = {fastInfo freeInfo slowInfo};
compare_jntanglesFromMotSinglePlot(subject, fnames, fnameMeasured, tInfo, figHandleArray, figContents, ref_dataFormatDelaware);
clear fnames fnameMeasured;

%
% Joint moments
%

% Get figure numbers and plot properties
figContentsAll = ref_jointMomentPlotLabelsSinglePlotSeparate;
allJointMoments = {'pelvis_tx' 'pelvis_list' 'pelvis_ty' 'pelvis_rotation' 'pelvis_tz' ...
    'pelvis_tilt' 'hip_flexion_l' 'hip_flexion_r' 'hip_adduction_l' 'hip_adduction_r' ...
    'hip_rotation_l' 'hip_rotation_r' 'knee_angle_l' 'knee_angle_r' 'ankle_angle_l' ...
    'ankle_angle_r' 'lumbar_extension' 'lumbar_bending' 'lumbar_rotation'};
figIndices = [];
for i = 1:length( desiredJointMoments )
    indicesOfCurrentJointMoment = find( strcmp( desiredJointMoments{i}, allJointMoments ) );
    figIndices = [figIndices indicesOfCurrentJointMoment];
end
figContents = figContentsAll( figIndices );
numberOfFiguresPerSpeed = length( figContents );
% Currently figContents contains only fast speed figure info.  Add info for
% free speed now.
for i = numberOfFiguresPerSpeed+1 : 2*numberOfFiguresPerSpeed
    figContents(i) = figContents(i-numberOfFiguresPerSpeed);
end
% And now, add figure info for slow speed.
for i = 2*numberOfFiguresPerSpeed+1 : 3*numberOfFiguresPerSpeed
    figContents(i) = figContents(i-numberOfFiguresPerSpeed);
end

fnames = { fullfile( de2dirs{FAST}, [subject '_' fastInfo.trial '_packaged.mot' ] )
           fullfile( de2dirs{FREE}, [subject '_' freeInfo.trial '_packaged.mot' ] )
           fullfile( de2dirs{SLOW}, [subject '_' slowInfo.trial '_packaged.mot' ] ) };
tInfo = {fastInfo freeInfo slowInfo};
% For non-separate compare_jointMomFromMotSinglePlot:
%figHandleArray = currentFigNumberLimit + 1 : currentFigNumberLimit + length( figIndices );
%currentFigNumberLimit = currentFigNumberLimit + length( figIndices );
figHandleArray = currentFigNumberLimit + 1 : currentFigNumberLimit + length( figIndices ) * 3;
currentFigNumberLimit = currentFigNumberLimit + length( figIndices ) * 3;
compare_jointMomFromMotSinglePlotSeparate(subject, fnames, tInfo, figHandleArray, figContents, ref_dataFormatDelaware);

%
% Muscle activations
%

% Get figure numbers and plot properties
figContentsAll = ref_muscleActPlotLabelsDelawareSinglePlot;
allMuscleActivations = {'glut_max_l' 'glut_max_r' 'glut_med_l' 'glut_med_r' ...
    'glut_min_l' 'glut_min_r' 'iliacus_l' 'iliacus_r' 'sar_l' 'sar_r' ...
    'tfl_l' 'tfl_r' 'vas_l' 'vas_r' 'rect_fem_l' 'rect_fem_r' 'semimem_l' ...
    'semimem_r' 'bifemlh_l' 'bifemlh_r' 'bifemsh_l' 'bifemsh_r' 'add_brev_l' ...
    'add_brev_r' 'add_mag_l' 'add_mag_r' 'pect_l' 'pect_r' 'quad_fem_l' ...
    'quad_fem_r' 'gem_l' 'gem_r' 'peri_l' 'peri_r' 'gas_l' 'gas_r' 'soleus_l' ...
    'soleus_r' 'flex_dig_l' 'flex_dig_r' 'tib_ant_l' 'tib_ant_r' 'ext_dig_l' ...
    'ext_dig_r' 'per_long_l' 'per_long_r' 'ercspn_l' 'ercspn_r' 'intobl_l' ...
    'intobl_r' 'extobl_l' 'extobl_r'};
figIndices = [];
for i = 1:length( desiredMuscleActivations )
    indicesOfCurrentMuscleActivation = find( strcmp( desiredMuscleActivations{i}, allMuscleActivations ) );
    figIndices = [figIndices indicesOfCurrentMuscleActivation];
end
figContents = figContentsAll( figIndices );

fnameMeasured = {};
figHandleArray = currentFigNumberLimit + 1 : currentFigNumberLimit + length( figIndices );
compare_muscleActFromMotDirectOverlaySinglePlot(subject, fnames, fnameMeasured, tInfo, figHandleArray, figContents, ref_dataFormatDelaware);
clear fnames fnameMeasured;

if ~GLOBAL_individualprintmenus
	combinedprintmenu(GLOBAL_figHandles, [subject '_3speeds']);
end
