function make_plotsOfJointMoments()

%
% Pick speed.
%

FAST = 1;
FREE = 2;
SLOW = 3;

speed = FAST;

%
% Set input file names.
%

switch speed
    case FAST
        dir = 'D:\programfiles\FCA\SU\Testing\delaware2_1.5_fast_auto';
        ikMomentsFile = fullfile( dir, 'ResultsCMC/de2_fast_walk1_ik_InverseDynamics_force.sto' );
        cmcMomentsFile = fullfile( dir, 'ResultsCMC/de2_fast_walk1_cmc_InverseDynamics_force.sto' );
        plotSettings.yAxisRange = [-100 200];
        plotSettings.yAxisTicks = -100 : 100 : 200;
        plotSettings.curveColors = { [ 0.5 0.25 0 ] [ 1 0.5 0 ] };
    case FREE
        dir = 'D:\programfiles\FCA\SU\Testing\delaware2_1.5_ss_auto_OLD';
        ikMomentsFile = fullfile( dir, 'ResultsCMC/de2_ss_walk1_ik_InverseDynamics_force.sto' );
        cmcMomentsFile = fullfile( dir, 'ResultsCMC/de2_ss_walk1_cmc_InverseDynamics_force.sto' );
        plotSettings.yAxisRange = [-100 200];
        plotSettings.yAxisTicks = -100 : 100 : 200;
        plotSettings.curveColors = { [ 0 0.5 0.25 ] [ 0 1 0.5 ] };
    case SLOW
        dir = 'D:\programfiles\FCA\SU\Testing\delaware2_1.5_vslow_auto';
        ikMomentsFile = fullfile( dir, 'ResultsCMC/de2_vslow_walk1_ik_InverseDynamics_force.sto' );
        cmcMomentsFile = fullfile( dir, 'ResultsCMC/de2_vslow_walk1_cmc_InverseDynamics_force.sto' );
        plotSettings.yAxisRange = [-100 200];
        plotSettings.yAxisTicks = -100 : 100 : 200;
        plotSettings.curveColors = { [ 0 0.25 0.5 ] [ 0 0.5 1 ] };
end

%
% Below we show an example of how to make a single plot containing multiple
% curves with desired line types, line colors, axis titles, axis labels, tick
% mark spacing, with data from various columns in multiple files.
%

% Set the figure (window) properties.
plotSettings.figureNumber = 1;
plotSettings.figurePosition = [ 2 2 3.25 2.25 ];
plotSettings.figureUnits = 'Inches';
plotSettings.figureBgColor = 'w';
plotSettings.figureFgColor = 'k';

% Set the plot properties (e.g., axes, labels).
plotSettings.yAxisLabel = 'Moment (N m)';
plotSettings.zeroLineOn = true;
plotSettings.zeroLineWidth = 0.25;
plotSettings.zeroLineColor = [ 0.3 0.3 0.3 ];
plotSettings.axisFontName = 'Times New Roman';
plotSettings.axisFontSize = 8;
plotSettings.axisTickDirection = 'Out';

% Set the time axis properties.
% If plotSettings.computeTimeLimitsAndTicksAutomatically is false, then
% plotSettings.xAxisRange and plotSettings.xAxisTicks should be set
% manually here by the user.
% If plotSettings.computeVerticalAxisLimitsAndTicksAutomatically is false,
% then plotSettings.yAxisRange and plotSettings.yAxisTicks should be set
% manually here by the user.
%plotSettings.timeOrPercent = 'Time';
%plotSettings.xAxisLabel = 'Time (s)';
plotSettings.timeOrPercent = 'Percent';
plotSettings.xAxisLabel = 'Percent of gait cycle';
plotSettings.shiftPercentAxisBy = 0;
plotSettings.computeTimeLimitsAndTicksAutomatically = false;
plotSettings.xAxisRange = [0 150];
plotSettings.xAxisTicks = 0 : 25 : 150;
plotSettings.computeVerticalAxisLimitsAndTicksAutomatically = false;

% Set the trial info for the plot.
currentDirectory = cd;
cd( dir );
switch speed
    case FAST
        [ sInfo, trialInfo ] = ref_trialInfoDelaware2( 'de2', 'fast_walk1' );
    case FREE
        [ sInfo, trialInfo ] = ref_trialInfoDelaware2( 'de2', 'ss_walk1' );
    case SLOW
        [ sInfo, trialInfo ] = ref_trialInfoDelaware2( 'de2', 'vslow_walk1' );
end
cd( currentDirectory );
trialInfo.gcLimb = 'L';
switch speed
    case FAST
        trialInfo.FP = trialInfo.fast_walk1.FP;
        trialInfo.limb = trialInfo.fast_walk1.limb;
        trialInfo.ictoMatrix = trialInfo.fast_walk1.ictoMatrix;
    case FREE
        trialInfo.FP = trialInfo.ss_walk1.FP;
        trialInfo.limb = trialInfo.ss_walk1.limb;
        trialInfo.ictoMatrix = trialInfo.ss_walk1.ictoMatrix;
    case SLOW
        trialInfo.FP = trialInfo.vslow_walk1.FP;
        trialInfo.limb = trialInfo.vslow_walk1.limb;
        trialInfo.ictoMatrix = trialInfo.vslow_walk1.ictoMatrix;
end
trialInfo.analogRate = 600;
trialInfo.tZeroAtFirstIC = 0;
% Get analog frames corresponding to IC and TO events from tInfo,
% and store in structure array.
for fpHitNum = 1:length(trialInfo.FP)
    ictoEvents(fpHitNum).ic  = trialInfo.ictoMatrix(fpHitNum, 1);
    ictoEvents(fpHitNum).oto = trialInfo.ictoMatrix(fpHitNum, 2);
    ictoEvents(fpHitNum).oic = trialInfo.ictoMatrix(fpHitNum, 3);
    ictoEvents(fpHitNum).to  = trialInfo.ictoMatrix(fpHitNum, 4);
    ictoEvents(fpHitNum).icNext = trialInfo.ictoMatrix(fpHitNum, 5);
end
plotSettings.ictoEvents = ictoEvents;
plotSettings.trialInfo = trialInfo;

% Set the curve properties.
plotSettings.curveStyles = { '-' '-' };
plotSettings.curveWidths = {  3   1  };
plotSettings.curveLabels = { 'IK' 'CMC' };
plotSettings.curveSourceFiles = { ...
    ikMomentsFile ...
    cmcMomentsFile };
plotSettings.curveSourceColumnLabels = { 'knee_angle_l' 'knee_angle_l' };
plotSettings.curveRepeatedSourceColumnNumbers = { 1 1 };

% Set the plot title properties.
plotSettings.figureTitle = 'Knee extension moments';
plotSettings.titleFontName = 'Times New Roman';
plotSettings.titleFontSize = 10;
plotSettings.titleVerticalAlignment = 'Middle';

% Plot the curves from the specified files and columns with the specified
% properties.
plot_dataFromMotOrStoFiles( plotSettings );
