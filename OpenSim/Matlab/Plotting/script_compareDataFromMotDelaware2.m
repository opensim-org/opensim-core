function script_compareDataFromMotDelaware2(subjectname, trialname, timeAxisLabel, bgColor, fgColor, gcLimb, individualprintmenus)
%
% Add comment here describing all parameters, and what function does!
%

if nargin < 3
	timeAxisLabel = 'time (s)';
end
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

global GLOBAL_individualprintmenus GLOBAL_figHandles;
GLOBAL_figHandles = [];
GLOBAL_individualprintmenus = individualprintmenus;

[sInfo, tInfo] = ref_trialInfoDelaware2(subjectname, trialname); % customize for subject
trial = tInfo.(trialname);
ref_dataFormat = ref_dataFormatDelaware;

% Directory containing <subject>_<trialname>EmgEnv.mot
%datadir = strcat('D:\programfiles\FCA\SU\Testing\delaware\delaware2\',trialname,'\');
datadir = [cd '\'];

subject = sInfo.subject;
ss.mass = sInfo.mass;
ss.analogRate = ref_dataFormat.analogRate;

eval(sprintf('trial = tInfo.%s;', trialname));

ss.trial = trial.trial;
ss.speed = trial.speed;
ss.ictoMatrix = trial.ictoMatrix;
ss.FP = trial.FP;
ss.limb = trial.limb;
ss.timeAxisLabel = timeAxisLabel;
ss.gcLimb = gcLimb;
ss.bgColor = bgColor;
ss.fgColor = fgColor;

% joint angles
fnames = {sprintf('%s_%s_ik.mot', subject, ss.trial)
		  sprintf('ResultsCMC/%s_%s_Kinematics_q.mot', subject, ss.trial)};
%fnameMeasured = sprintf('%s_%s.mot', subject, ss.trial);
fnameMeasured = {};
figHandleArray = 1:3;
compare_jntanglesFromMot(subject, fnames, fnameMeasured, ss, figHandleArray, ref_dataFormatDelaware);
clear fnames fnameMeasured;

% GRFs
%fnames = {sprintf('%s_%s_ik.mot', subject, ss.trial)};
%fnameMeasured = sprintf('%s_%s.mot', subject, ss.trial);
%figHandleArray = 8:9;
%compare_grftFromMot(subject, fnames, fnameMeasured, ss, figHandleArray);
%clear fnames fnameMeasured;

% muscle excitations, activations, and forces; actuator forces; joint moments
fnames = {sprintf('%s_%s_packaged.mot', subject, ss.trial)};
% Uncomment the line below, and comment out the line below that, to plot
% measured EMG data against the muscle activations.
%fnameMeasured = sprintf('%s%s_%sEmgEnv.mot', datadir, subject, ss.trial);
fnameMeasured = {};
figHandleArray = 10:18;
compare_muscleActFromMotDirectOverlay(subject, fnames, fnameMeasured, ss, figHandleArray, ref_dataFormatDelaware);
%figHandleArray = 20:28;
%compare_muscleExcFromMot(subject, fnames, ss, figHandleArray, ref_dataFormatDelaware);
%figHandleArray = 30:38;
%compare_muscleFrcFromMot(subject, fnames, ss, figHandleArray, ref_dataFormatDelaware);
%figHandleArray = 40:43;
%compare_actuatorFrcFromMot(subject, fnames, ss, figHandleArray, ref_dataFormatDelaware);
figHandleArray = 50:53;
% Change "false" to "true" in the line below to plot the literature joint moments
% against the simulated joint moments and joint moments estimated from
% applying inverse dynamics to the result of applying inverse kinematics to
% the measured marker trajectories.
compare_jointMomFromMot(subject, fnames, false, ss, figHandleArray, ref_dataFormatDelaware);
clear fnames fnameMeasured;

if ~GLOBAL_individualprintmenus
	combinedprintmenu(GLOBAL_figHandles, [subject '_' ss.trial]);
end
