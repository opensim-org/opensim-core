% script_compareDataFromMot900045.m
% Purpose:  This script calls functions that plot joint angles, 
%           joint moments, GRFs and COPs, muscle excitations, 
%           muscle activations, and muscle forces from *.mot files, 
%           for comparing the performance of UW-Gait simulation routines.
%
% Format for Subject/File Information:
%       subject is the 6-digit subject ID ('character array')
%       fnames is a cell array of file names corresponding to 
%           the workflow-related motion files to be compared
%       fnameMeasured is the name of a motion file containing measured data
%           (joint angles, joint moments, or EMG) for comparison
%       tInfo is a structure containing the following 'trial info':
%           *.mass  - mass of subject (not needed here...)
%           *.speed  - average walking speed during trial in m/s
%           *.FP    - FP #s in the order hit (cell array) 
%           *.limb  - limbs corresponding to FP strikes (cell array)
%           *.ictoMatrix - matrix of ic, oto, oic, to, & icNext events, 
%                               in analog frames, one row per FP hit.
%           *.analogRate - analog sample rate
%
% Available Functions:
%  compare_jntanglesStatic(subject, fnames, fnameMeasured, figHandleArray)
%  compare_jntanglesFromMot(subject, fnames, fnameMeasured, tInfo, figHandleArray)
%  compare_jntmomentsFromMot(subject, fnames, tInfo, figHandleArray)
%  compare_jntmomentsSagittal(subject, fnames, fnameMeasured, tInfo, figHandleArray)
%  compare_grftFromMot(subject, fnames, fnameMeasured, tInfo, figHandleArray)
%  compare_muscleActFromMot(subject, fnames, fnameMeasured, tInfo, figHandleArray)
%  compare_muscleExcFromMot(subject, fnames, tInfo, figHandleArray)
%  compare_muscleFrcFromMot(subject, fnames, tInfo, figHandleArray)
%          
%  NOTE:  These functions compare data from different solutions 
%                  for a single trial, not different trials.
%
% ASA, 11-05; revised 2-06




%%% SELF-SELECTED SPEED WALKING TRIAL
subject = '900045';
ss.mass = 66.0;
ss.speed = 1.12;
ss.FP = {1 2 3 4};
ss.limb = {'R', 'L', 'R' ,'L'};
ss.ictoMatrix = [  384   522   985  1154  1601; ...
                   985  1154  1601  1758  2254; ...
                  1601  1758  2254  2385  2844; ...
                  2254  2385  2844  2990  3474];
ss.analogRate = 1080;

% joint angles
fnames = {'900045_15cmc_minCnstr.mot'        % my CMC soln w/ minimum exc constraints
          '900045_15ik_fromClayChopped.mot'  % IK from Clay
          '900045_15rea_fromClay.mot'        % my REA solution, using Clay's IK
          '900045_15cmc_fromClay.mot'};      % my CMC solution, using Clay's IK
fnameMeasured = '900045_15.mot';    
figHandleArray = 1:3;
compare_jntanglesFromMot(subject, fnames, fnameMeasured, ss, figHandleArray);
clear fnames fnameMeasured;

% joint moments
fnames = {'900045_15cmcID_minCnstr.mot'
          '900045_15ikID_fromClay.mot'
          '900045_15reaID_fromClay.mot'
          '900045_15cmcID_fromClay.mot'};
fnameMeasured = '900045_15Moments.mot';
figHandleArray = 4:6;
compare_jntmomentsFromMot(subject, fnames, ss, figHandleArray);
figHandleArray = 7;
compare_jntmomentsSagittal(subject, fnames, fnameMeasured, ss, figHandleArray);
clear fnames fnameMeasured;

% GRFs
fnames = {'900045_15cmc_minCnstr.mot'
          '900045_15ik_fromClayChopped.mot'
          '900045_15rea_fromClay.mot'
          '900045_15cmc_fromClay.mot'};
fnameMeasured = '900045_15.mot';       
figHandleArray = 8:9;
compare_grftFromMot(subject, fnames, fnameMeasured, ss, figHandleArray);
clear fnames fnameMeasured;

% muscle excitations, activations, and forces
fnames = {'900045_15cmc_minCnstr.mot'
          '900045_15cmc_fromClay.mot'};
fnameMeasured = '900045_15EmgEnv.mot';
figHandleArray = 10:18;
compare_muscleActFromMot(subject, fnames, fnameMeasured, ss, figHandleArray);
figHandleArray = 20:28;
compare_muscleExcFromMot(subject, fnames, ss, figHandleArray);
figHandleArray = 30:38;
compare_muscleFrcFromMot(subject, fnames, ss, figHandleArray);
clear fnames fnameMeasured;


