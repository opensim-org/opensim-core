function [sInfo, tInfo] = ref_trialInfoDelaware()

sInfo.subject = 'delaware3';
sInfo.mass = 72.6;

% ss_walking1
trial.trial = 'ss_walking1';
trial.speed = 1.13;
trial.contactRanges = [    0  490
						 360  855
						 735 1225
						1100 1595
						1465 1955
						1835 2320
						2195 2690
						2565 3060
						2930 3430
						3305 3795
						3675 4155
						4035 4540
						4405 4890
						4765 5265
						5135 5620
						5505 5995
						5865 6360
						6225 6725
						6600 7095
						6965 7475
						7345 7835
						7710 8215
						8085 8575
						8460 8935
						8820 0];
[trial.ictoMatrix, trial.FP, trial.limb] = build_delaware_ictoMatrix(trial.contactRanges, 'R', 1);
trial.ancFile = 'SS_walking1.anc';
trial.c3dFile = 'SS_walking1_smoothed.c3d';
tInfo.ss_walking1 = trial;

% slow_walking_50_1
trial.trial = 'slow_walking_50_1';
trial.speed = 0.565;
trial.contactRanges = [ ...
           0         581
         334        1068
         858        1602
        1351        2120
        1901        2658
        2444        3160
        2962        3731
        3495        4247
        4037        4771
        4535        5285
        5080        5844
        5598        6338
        6133        6855
        6644        7394
        7187        7905
        7686        8432
        8216           0];
[trial.ictoMatrix, trial.FP, trial.limb] = build_delaware_ictoMatrix(trial.contactRanges, 'L', 2);
trial.ancFile = 'slow_walking_50_1.anc';
trial.c3dFile = 'slow_walking_50_1_smoothed.c3d';
tInfo.slow_walking_50_1 = trial;

% slow_walking_75_1
trial.trial = 'slow_walking_75_1';
trial.speed = 0.848;
ictoInput = compute_ictoMatrixInput('../slow_walking_75_1/delaware3_slow_walking_75_1_grf.mot');
trial.contactRanges = ictoInput.contactRanges;
[trial.ictoMatrix, trial.FP, trial.limb] = build_delaware_ictoMatrix(trial.contactRanges, ictoInput.firstICLimb, ictoInput.firstICFP);
trial.ancFile = 'slow_walking_75_1.anc';
trial.c3dFile = 'slow_walking_75_1_smoothed.c3d';
tInfo.slow_walking_75_1 = trial;

% fast_walking_1
trial.trial = 'fast_walking_1';
trial.speed = 2.06;
trial.contactRanges = [ ...
           0         188
          85         449
         369         717
         641         992
         921        1277
        1204        1553
        1485        1839
        1765        2111
        2046        2402
        2321        2692
        2609        2946
        2874        3219
        3152        3496
        3419        3766
        3700        4055
        3973        4314
        4246        4596
        4523        4872
        4800        5165
        5078        5441
        5360        5705
        5631           0];
[trial.ictoMatrix, trial.FP, trial.limb] = build_delaware_ictoMatrix(trial.contactRanges, 'L', 2);
trial.ancFile = 'fast_walking_1.anc';
trial.c3dFile = 'fast_walking_1_smoothed.c3d';
tInfo.fast_walking_1 = trial;
