function script_compareDataFromMotDelaware2(subjectname, trialname, individualprintmenus)

if nargin < 2
	trialname = 'ss_walk1';
end

if nargin < 3
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

% joint angles
fnames = {sprintf('%s_%s_ik.mot', subject, ss.trial)
		  sprintf('Results/%s_%s_RRA2_Kinematics_q.mot', subject, ss.trial)};
fnameMeasured = sprintf('%s_%s.mot', subject, ss.trial);
figHandleArray = 1:3;
compare_jntanglesFromMot(subject, fnames, fnameMeasured, ss, figHandleArray, ref_dataFormatDelaware);
clear fnames fnameMeasured;

% GRFs
fnames = {sprintf('%s_%s_ik.mot', subject, ss.trial)};
fnameMeasured = sprintf('%s_%s.mot', subject, ss.trial);
figHandleArray = 8:9;
compare_grftFromMot(subject, fnames, fnameMeasured, ss, figHandleArray);
clear fnames fnameMeasured;

% muscle excitations, activations, and forces
fnames = {sprintf('%s_%s_packaged.mot', subject, ss.trial)};
fnameMeasured = sprintf('%s%s_%sEmgEnv.mot', datadir, subject, ss.trial);
figHandleArray = 10:18;
compare_muscleActFromMotDirectOverlay(subject, fnames, fnameMeasured, ss, figHandleArray, ref_dataFormatDelaware);
figHandleArray = 20:28;
compare_muscleExcFromMot(subject, fnames, ss, figHandleArray, ref_dataFormatDelaware);
figHandleArray = 30:38;
compare_muscleFrcFromMot(subject, fnames, ss, figHandleArray, ref_dataFormatDelaware);
figHandleArray = 40:43;
compare_actuatorFrcFromMot(subject, fnames, ss, figHandleArray, ref_dataFormatDelaware);
clear fnames fnameMeasured;

if ~GLOBAL_individualprintmenus
	combinedprintmenu(GLOBAL_figHandles, [subject '_' ss.trial]);
end
