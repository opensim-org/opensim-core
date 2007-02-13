function [sInfo, tInfo] = ref_trialInfoDelaware2()

sInfo.subject = 'delaware2';
sInfo.mass = 65.9;

SS_WALKING1 = 1;
VERY_SLOW_WALKING1 = 2;
FAST_WALKING1 = 3;

trialChoice = FAST_WALKING1;

% ss_walking1
if trialChoice == SS_WALKING1
    trial.trial = 'ss_walking1';
    trial.speed = 1.36;
    ictoInput = compute_ictoMatrixInput('delaware2_ss_walking1_newgrfprocessing.mot');
    trial.contactRanges = ictoInput.contactRanges;
    [trial.ictoMatrix, trial.FP, trial.limb] = build_delaware_ictoMatrix(ictoInput.contactRanges, ictoInput.firstICLimb, ictoInput.firstICFP);
    trial.ancFile = 'SS_walking1.anc';
    trial.c3dFile = 'SS_walking1_smoothed.c3d';
    tInfo.ss_walking1 = trial;
end

% very_slow_walking1
if trialChoice == VERY_SLOW_WALKING1
    trial.trial = 'very_slow_walking1';
    trial.speed = 0.680;
    ictoInput = compute_ictoMatrixInput('delaware2_very_slow_walking1_newgrfprocessing.mot');
    trial.contactRanges = ictoInput.contactRanges;
    [trial.ictoMatrix, trial.FP, trial.limb] = build_delaware_ictoMatrix(trial.contactRanges, ictoInput.firstICLimb, ictoInput.firstICFP);
    trial.ancFile = 'very_slow_walking_501.anc';
    trial.c3dFile = 'very_slow_walking_501_smoothed_trimmed.c3d';
    tInfo.very_slow_walking1 = trial;
end

% fast_walking1
if trialChoice == FAST_WALKING1
    trial.trial = 'fast_walking1';
    trial.speed = 2.320;
    ictoInput = compute_ictoMatrixInput('delaware2_fast_walking1_newgrfprocessing.mot');
    trial.contactRanges = ictoInput.contactRanges;
    [trial.ictoMatrix, trial.FP, trial.limb] = build_delaware_ictoMatrix(trial.contactRanges, ictoInput.firstICLimb, ictoInput.firstICFP);
    trial.ancFile = 'fast_walking_2001_smoothed1.anc';
    trial.c3dFile = 'fast_walking_2001_smoothed1.c3d';
    tInfo.fast_walking1 = trial;
end
