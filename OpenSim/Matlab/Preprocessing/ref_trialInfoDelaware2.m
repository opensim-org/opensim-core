function [sInfo, tInfo] = ref_trialInfoDelaware2()

sInfo.subject = 'delaware2';
sInfo.mass = 65.9;

% ss_walking1
trial.trial = 'ss_walking1';
trial.speed = 1.36;
ictoInput = compute_ictoMatrixInput('delaware2_ss_walking1_newgrfprocessing.mot');
trial.contactRanges = ictoInput.contactRanges;
[trial.ictoMatrix, trial.FP, trial.limb] = build_delaware_ictoMatrix(ictoInput.contactRanges, ictoInput.firstICLimb, ictoInput.firstICFP);
trial.ancFile = 'SS_walking1.anc';
trial.c3dFile = 'SS_walking1_smoothed.c3d';
tInfo.ss_walking1 = trial;

% slow_walking_50_1
%trial.trial = 'slow_walking_50_1';
%trial.speed = 0.680;
%trial.contactRanges = [ ...
%           0         581
%         334        1068
%         858        1602
%        1351        2120
%        1901        2658
%        2444        3160
%        2962        3731
%        3495        4247
%        4037        4771
%        4535        5285
%        5080        5844
%        5598        6338
%        6133        6855
%        6644        7394
%        7187        7905
%        7686        8432
%        8216           0];
%[trial.ictoMatrix, trial.FP, trial.limb] = build_delaware_ictoMatrix(trial.contactRanges, 'L', 2);
%trial.ancFile = 'slow_walking_50_1.anc';
%trial.c3dFile = 'slow_walking_50_1_smoothed.c3d';
%tInfo.slow_walking_50_1 = trial;
%
% fast_walking_1
%trial.trial = 'fast_walking_1';
%trial.speed = 2.320;
%trial.contactRanges = [ ...
%           0         188
%          85         449
%         369         717
%         641         992
%         921        1277
%        1204        1553
%        1485        1839
%        1765        2111
%        2046        2402
%        2321        2692
%        2609        2946
%        2874        3219
%        3152        3496
%        3419        3766
%        3700        4055
%        3973        4314
%        4246        4596
%        4523        4872
%        4800        5165
%        5078        5441
%        5360        5705
%        5631           0];
%[trial.ictoMatrix, trial.FP, trial.limb] = build_delaware_ictoMatrix(trial.contactRanges, 'L', 2);
%trial.ancFile = 'fast_walking_1.anc';
%trial.c3dFile = 'fast_walking_1_smoothed.c3d';
%tInfo.fast_walking_1 = trial;
