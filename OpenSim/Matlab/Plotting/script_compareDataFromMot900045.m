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


% %%% STATIC TRIAL
% subject = '900045';
% fnames = {'900045_2sc.mot'};
% fnameMeasured = '900045_2.mot';      
% figHandleArray = 1:3;
% compare_jntanglesStatic(subject, fnames, fnameMeasured, figHandleArray);
% clear fnames fnameMeasured;


%%% SELF-SELECTED SPEED WALKING TRIAL
subject = '900045';
ss.mass = 66.0;
ss.speed = 1.11;
ss.FP = {1 2 3 4};
ss.limb = {'R', 'L', 'R', 'L'};
ss.ictoMatrix = [  348   477   924  1059  1511; ...
                   924  1059  1511  1646  2127; ...
                  1511  1646  2127  2266  2754; ...
                  2127  2266  2754  2842  3303];
ss.analogRate = 1080;

% joint angles
fnames = {'900045_16ik.mot'
          '900045_16rea_lsw.mot'
          '900045_16cmc_lsw_uncnstr.mot'
          '900045_16cmc_lsw_cnstr.mot'};
fnameMeasured = '900045_16.mot';    
figHandleArray = 1:3;
%compare_jntanglesFromMot(subject, fnames, fnameMeasured, ss, figHandleArray, ref_dataFormatGillette);
clear fnames fnameMeasured;

% joint moments
fnames = {'900045_16ikID_lsw.mot'
          '900045_16reaID_lsw.mot'
          '900045_16cmcID_lsw_uncnstr.mot'
          '900045_16cmcID_lsw_cnstr.mot'};
fnameMeasured = '900045_16Moments.mot';
figHandleArray = 4:6;
%compare_jntmomentsFromMot(subject, fnames, ss, figHandleArray);
figHandleArray = 7;
%compare_jntmomentsSagittal(subject, fnames, fnameMeasured, ss, figHandleArray, ref_dataFormatGillette);
clear fnames fnameMeasured;

% GRFs
fnames = {'900045_16ik.mot'
          '900045_16rea_lsw.mot'
          '900045_16cmc_lsw_uncnstr.mot'
          '900045_16cmc_lsw_cnstr.mot'};
fnameMeasured = '900045_16.mot';       
figHandleArray = 8:9;
compare_grftFromMot(subject, fnames, fnameMeasured, ss, figHandleArray);
clear fnames fnameMeasured;

% muscle excitations, activations, and forces
fnames = {'900045_16cmc_lsw_uncnstr.mot'
          '900045_16cmc_lsw_cnstr.mot'};
fnameMeasured = '900045_16EmgEnv.mot';
figHandleArray = 10:18;
compare_muscleActFromMot(subject, fnames, fnameMeasured, ss, figHandleArray, ref_dataFormatGillette);
figHandleArray = 20:28;
compare_muscleExcFromMot(subject, fnames, ss, figHandleArray);
figHandleArray = 30:38;
compare_muscleFrcFromMot(subject, fnames, ss, figHandleArray);
clear fnames fnameMeasured;


% %%% FAST WALKING TRIAL
% subject = '900045';
% fast.mass = 66.0;
% fast.speed = 1.45;
% fast.FP = {3 4};
% fast.limb = {'L', 'R'};
% fast.ictoMatrix = [823   936  1364  1475  1881; ...
%                   1364  1475  1881  2002  2412];
% fast.analogRate = 1080;
% 
% % joint angles
% fnames = {'900045_27ik.mot'
%           '900045_27rea_lsw.mot'
%           '900045_27cmc_lsw_uncnstr.mot'
%           '900045_27cmc_lsw_cnstr.mot'};
% fnameMeasured = '900045_27.mot';
% figHandleArray = 1:3;
% compare_jntanglesFromMot(subject, fnames, fnameMeasured, fast, figHandleArray);
% clear fnames fnameMeasured;
% 
% % joint moments
% fnames = {'900045_27ikID_lsw.mot'
%           '900045_27reaID_lsw.mot'
%           '900045_27cmcID_lsw_uncnstr.mot'
%           '900045_27cmcID_lsw_cnstr.mot'};
% fnameMeasured = '900045_27Moments.mot';
% figHandleArray = 4:6;
% compare_jntmomentsFromMot(subject, fnames, fast, figHandleArray);
% figHandleArray = 7;
% compare_jntmomentsSagittal(subject, fnames, fnameMeasured, fast, figHandleArray);
% clear fnames fnameMeasured;
% 
% % GRFs
% fnames = {'900045_27ik.mot'
%           '900045_27rea_lsw.mot'
%           '900045_27cmc_lsw_uncnstr.mot'
%           '900045_27cmc_lsw_cnstr.mot'};
% fnameMeasured = '900045_27.mot';
% figHandleArray = 8:9;
% compare_grftFromMot(subject, fnames, fnameMeasured, fast, figHandleArray);
% clear fnames fnameMeasured;
% 
% % muscle excitations, activations, and forces
% fnames = {'900045_27cmc_lsw_uncnstr.mot'
%           '900045_27cmc_lsw_cnstr.mot'};
% fnameMeasured = '900045_27EmgEnv.mot';
% figHandleArray = 10:18;
% compare_muscleActFromMot(subject, fnames, fnameMeasured, fast, figHandleArray);
% figHandleArray = 20:28;
% compare_muscleExcFromMot(subject, fnames, fast, figHandleArray);
% figHandleArray = 30:38;
% compare_muscleFrcFromMot(subject, fnames, fast, figHandleArray);
% clear fnames fnameMeasured;
% 
% 
% %%% SLOW WALKING TRIAL 
% subject = '900045';
% slow.mass = 66.0;
% slow.speed = 0.71;
% slow.FP = {1 2};
% slow.limb = {'L', 'R'};
% slow.ictoMatrix = [741   927  1499  1766  2268; ...
%                   1499  1766  2268  2455  2979];
% slow.analogRate = 1080;
% 
% % joint angles
% fnames = {'900045_18ik_lsw.mot'
%           '900045_18rea_lsw.mot'
%           '900045_18cmc_lsw_uncnstr.mot'
%           '900045_18cmc_lsw_cnstr.mot'};
% fnameMeasured = '900045_18.mot';
% figHandleArray = 1:3;
% compare_jntanglesFromMot(subject, fnames, fnameMeasured, slow, figHandleArray);
% clear fnames fnameMeasured;
% 
% % joint moments
% fnames = {'900045_18ikID_lsw.mot'
%           '900045_18reaID_lsw.mot'
%           '900045_18cmcID_lsw_uncnstr.mot'
%           '900045_18cmcID_lsw_cnstr.mot'};
% fnameMeasured = '900045_18Moments.mot';
% figHandleArray = 4:6;
% compare_jntmomentsFromMot(subject, fnames, slow, figHandleArray);
% figHandleArray = 7;
% compare_jntmomentsSagittal(subject, fnames, fnameMeasured, slow, figHandleArray);
% clear fnames fnameMeasured;
% 
% % GRFs
% fnames = {'900045_18ik.mot'
%           '900045_18rea_lsw.mot'
%           '900045_18cmc_lsw_uncnstr.mot'
%           '900045_18cmc_lsw_cnstr.mot'};
% fnameMeasured = '900045_18.mot';
% figHandleArray = 8:9;
% compare_grftFromMot(subject, fnames, fnameMeasured, slow, figHandleArray);
% clear fnames fnameMeasured;
% 
% % muscle excitations, activations, and forces
% fnames = {'900045_18cmc_lsw_uncnstr.mot'
%           '900045_18cmc_lsw_cnstr.mot'};
% fnameMeasured = '900045_18EmgEnv.mot';
% figHandleArray = 10:18;
% compare_muscleActFromMot(subject, fnames, fnameMeasured, slow, figHandleArray);
% figHandleArray = 20:28;
% compare_muscleExcFromMot(subject, fnames, slow, figHandleArray);
% figHandleArray = 30:38;
% compare_muscleFrcFromMot(subject, fnames, slow, figHandleArray);
% clear fnames fnameMeasured;
% 
% 
% %%% X-SLOW WALKING TRIAL
% subject = '900045';
% xslow.mass = 66.0;
% xslow.speed = 0.49;
% xslow.FP = {3 4};
% xslow.limb = {'L', 'R'};
% xslow.ictoMatrix = [3565  4005  4620  5036  5643; ...
%                     4620  5036  5643  6139  6660];
% xslow.analogRate = 1080;
% 
% % joint angles
% fnames = {'900045_26ik.mot'
%           '900045_26rea_lsw.mot'
%           '900045_26cmc_lsw_uncnstr.mot'
%           '900045_26cmc_lsw_cnstr.mot'};
% fnameMeasured = '900045_26.mot';
% figHandleArray = 1:3;
% compare_jntanglesFromMot(subject, fnames, fnameMeasured, xslow, figHandleArray);
% clear fnames fnameMeasured;
% 
% % joint moments
% fnames = {'900045_26ikID_lsw.mot'
%           '900045_26reaID_lsw.mot'
%           '900045_26cmcID_lsw_uncnstr.mot'
%           '900045_26cmcID_lsw_cnstr.mot'};
% fnameMeasured = '900045_26Moments.mot';
% figHandleArray = 4:6;
% compare_jntmomentsFromMot(subject, fnames, xslow, figHandleArray);
% figHandleArray = 7;
% compare_jntmomentsSagittal(subject, fnames, fnameMeasured, xslow, figHandleArray);
% clear fnames fnameMeasured;
% 
% % GRFs
% fnames = {'900045_26ik.mot'
%           '900045_26rea_lsw.mot'
%           '900045_26cmc_lsw_uncnstr.mot'
%           '900045_26cmc_lsw_cnstr.mot'};
% fnameMeasured = '900045_26.mot';
% figHandleArray = 8:9;
% compare_grftFromMot(subject, fnames, fnameMeasured, xslow, figHandleArray);
% clear fnames fnameMeasured;
% 
% % muscle excitations, activations, and forces
% fnames = {'900045_26cmc_lsw_uncnstr.mot'
%           '900045_26cmc_lsw_cnstr.mot'};
% fnameMeasured = '900045_26EmgEnv.mot';
% figHandleArray = 10:18;
% compare_muscleActFromMot(subject, fnames, fnameMeasured, xslow, figHandleArray);
% figHandleArray = 20:28;
% compare_muscleExcFromMot(subject, fnames, xslow, figHandleArray);
% figHandleArray = 30:38;
% compare_muscleFrcFromMot(subject, fnames, xslow, figHandleArray);
% clear fnames fnameMeasured;
