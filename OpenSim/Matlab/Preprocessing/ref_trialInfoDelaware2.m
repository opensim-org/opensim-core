function [sInfo, tInfo] = ref_trialInfoDelaware2(subjectname, trialname)

sInfo.subject = subjectname;
sInfo.mass = 65.9;
trial.trial = trialname;

if ~strcmp(trialname,'ss_walk1') && ~strcmp(trialname,'vslow_walk1') && ~strcmp(trialname,'fast_walk1') && ~strcmp(trialname,'slow_walk1')
    sprintf('trialname %s not found',trialname);
end

ictoInput = compute_ictoMatrixInput( [subjectname '_' trialname '_grf.mot'] );
trial.contactRanges = ictoInput.contactRanges;
[trial.ictoMatrix, trial.FP, trial.limb] = build_delaware_ictoMatrix(ictoInput.contactRanges, ictoInput.firstICLimb, ictoInput.firstICFP);
trial.ancFile = [subjectname '_' trialname '.anc'];
trial.c3dFile = [subjectname '_' trialname '_smoothed.c3d'];

% ss_walking1
if strcmp(trialname,'ss_walk1')
    if strcmp(subjectname,'de2')
        trial.speed = 1.36;
    else
        trial.speed = 1.13;
    end
    tInfo.ss_walk1 = trial;
end

% slow_walking1
if strcmp(trialname,'slow_walk1')
    if strcmp(subjectname,'de2')
        trial.speed = 1.02;
    else
        trial.speed = 0.848;
    end
    tInfo.slow_walk1 = trial;
end

% very_slow_walking1
if strcmp(trialname,'vslow_walk1')
    if strcmp(subjectname,'de2')
        trial.speed = 0.680;
    else
        trial.speed = 0.565;
    end
    tInfo.vslow_walk1 = trial;
end

% fast_walking1
if strcmp(trialname,'fast_walk1')
    if strcmp(subjectname,'de2')
        trial.speed = 2.320;
    else
        trial.speed = 2.06;
    end
    tInfo.fast_walk1 = trial;
end
