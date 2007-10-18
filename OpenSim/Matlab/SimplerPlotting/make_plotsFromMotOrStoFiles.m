function make_plotsFromMotOrStoFiles()
%
% This is the main function to call in this directory.  This function contains
% calls to other plotting scripts that allow the user to plot any combination
% of quantities from .mot files from OpenSim or SIMM, and from .sto files from
% OpenSim, in any number of plots, with any desired figure, plot, and curve
% formats.
%
% CTJ, 09/2007
%

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
plotSettings.yAxisRange = [ -100 0 ];
plotSettings.yAxisTicks = -100 : 50 : 0;
plotSettings.yAxisLabel = 'Angle (degrees)';
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
%plotSettings.timeOrPercent = 'Time';
%plotSettings.xAxisLabel = 'Time (s)';
plotSettings.timeOrPercent = 'Percent';
plotSettings.xAxisLabel = 'Percent of gait cycle';
plotSettings.shiftPercentAxisBy = 0;
plotSettings.computeTimeLimitsAndTicksAutomatically = true;
plotSettings.computeVerticalAxisLimitsAndTicksAutomatically = true;

% Set the trial info for the plot.
currentDirectory = cd;
cd( 'D:\programfiles\FCA\SU\Testing\delaware2_1.5_ss_auto_simbody_cfsqp' );
[ sInfo, trialInfo ] = ref_trialInfoDelaware2( 'de2', 'ss_walk1' );
cd( currentDirectory );
trialInfo.gcLimb = 'L';
trialInfo.FP = trialInfo.ss_walk1.FP;
trialInfo.limb = trialInfo.ss_walk1.limb;
trialInfo.ictoMatrix = trialInfo.ss_walk1.ictoMatrix;
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
plotSettings.curveStyles = { '-' '-.' };
plotSettings.curveWidths = {  1   3  };
plotSettings.curveColors = { 'r' 'r' };
plotSettings.curveLabels = { 'IK' 'RRA' };
plotSettings.curveSourceFiles = { ...
    'D:\programfiles\FCA\SU\Testing\delaware2_1.5_ss_auto_simbody_cfsqp\de2_ss_walk1_ik.mot' ...
    'D:\programfiles\FCA\SU\Testing\delaware2_1.5_ss_auto_simbody_cfsqp\ResultsRRA\de2_ss_walk1_RRA_Kinematics_q.mot' };
plotSettings.curveSourceColumnLabels = { 'knee_angle_l' 'knee_angle_l' };
plotSettings.curveRepeatedSourceColumnNumbers = { 1 1 };

% Set the plot title properties.
plotSettings.figureTitle = 'Knee angles';
plotSettings.titleFontName = 'Times New Roman';
plotSettings.titleFontSize = 10;
plotSettings.titleVerticalAlignment = 'Middle';

% Plot the curves from the specified files and columns with the specified
% properties.
plot_dataFromMotOrStoFiles( plotSettings );
