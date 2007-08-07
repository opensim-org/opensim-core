function [] = compare_jointMomFromMot(subject, fnames, tInfo, figHandleArray, ref_dataFormat)
% Purpose:  Creates figure windows specified by figHandleArray and, 
%           for each file in fnames, overlays plots of:
%           (1) CMC-generated actuator forces vs time,
%                   for actuators specified in ref_actuatorFrcPlotLabels()
%           (2) L & R vertical GRF vs time, for reference
%         
% Input:    subject is a 6-digit subject ID ('character array')
%           fnames is a cell array of file names corresponding to 
%               the workflow-related motion files to be compared
%           tInfo is a structure containing the following 'trial info':
%               *.mass - subject mass (used to scale axes for force data)
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb - limbs corresponding to FP strikes (cell array)
%               *.ictoMatrix - matrix of ic, oto, oic, to, & icNext events, 
%                               in analog frames, one row per FP hit 
%               *.analogRate - analog sample rate
%           figHandleArray specifies the numbers of the figure windows
%               to be created.
%
% Called Functions:
%   read_motionFile(fname)
%   ref_jointMomentPlotLabels()
%   overlay_grftVertical(q, qPlotLabel, time, limb, fileNum)
%   overlay_jointMomentsFromMot(q, qPlotLabel, time, fileNum)
%   format_jointMomFromMot(fnames, timeRange, tInfo ...
%              subplotTitle, subplotAxisLabel, subplotRange, subplotTick)
%   Suptitle(titleString)
%
% CTJ, 07-07, adapted from:
% ASA, 11-05, revised 2-06

[q, time, timeRange, ictoEvents] = get_commonComparisonData('Joint Moments vs Time', subject, fnames, tInfo);

% Read in literature joint moment data from:
% - Crowninshield et al. (1978)
% - Cappozzo (1983)
% - Cappozzo et al. (1975)
% - Patriarco et al. (1981)
% - Inman et al. (1981)
% - Kadaba et al. (1989) - not used
% - MacKinnon and Winter (1993) - not used
momentDataArray = xlsread('LiteratureJointMoments.xls');
% Time arrays
cappozzoTimesH = momentDataArray(36:50, 8);
cappozzoTimesB = momentDataArray(36:52, 2);
gaitCycleTimes = momentDataArray( 4:25, 3);
% Back moments from 0% to 100% of a gait cycle
% From Cappozzo (1983)
origLumbarExtension = momentDataArray(36:50, 10);
origLumbarBending   = momentDataArray(36:52,  6);
origLumbarRotation  = momentDataArray(36:50, 12);
% Hip flexion moments from 0% to 100% of a gait cycle
% From Inman et al. 1981, Cappozzo et al. 1975, Crowninshield et al. 1978
leftHipFlexionInman         = momentDataArray(4:25, 32);
leftHipFlexionCappozzo      = momentDataArray(4:25, 15);
leftHipFlexionCrowninshield = momentDataArray(4:25,  9);
rightHipFlexionInman         = leftHipFlexionInman;
rightHipFlexionCappozzo      = leftHipFlexionCappozzo;
rightHipFlexionCrowninshield = leftHipFlexionCrowninshield;
% Hip adduction moments from 0% to 100% of a gait cycle
% From Crowninshield et al. 1978, Patriarco et al. 1981
leftHipAdductionCrowninshield = momentDataArray(4:25,  6);
leftHipAdductionPatriarco     = momentDataArray(4:25, 26);
rightHipAdductionCrowninshield = leftHipAdductionCrowninshield;
rightHipAdductionPatriarco     = leftHipAdductionPatriarco;
% Hip rotation moments from 0% to 100% of a gait cycle
% From Crowninshield et al. 1978, Patriarco et al. 1981
leftHipRotationCrowninshield = momentDataArray(4:25,  8);
leftHipRotationPatriarco     = momentDataArray(4:25, 25);
rightHipRotationCrowninshield = leftHipRotationCrowninshield;
rightHipRotationPatriarco     = leftHipRotationPatriarco;
% Knee flexion moments from 0% to 100% of a gait cycle
% From Cappozzo et al. 1975, Inman et al. 1981
leftKneeFlexionCappozzo = momentDataArray(4:25, 18);
leftKneeFlexionInman    = momentDataArray(4:25, 33);
rightKneeFlexionCappozzo = leftKneeFlexionCappozzo;
rightKneeFlexionInman    = leftKneeFlexionInman;
% Ankle flexion moments from 0% to 100% of a gait cycle
% Inman et al. 1981, Cappozzo et al. 1975
leftAnkleFlexionInman    = momentDataArray(4:25, 34);
leftAnkleFlexionCappozzo = momentDataArray(4:25, 21);
rightAnkleFlexionInman    = leftAnkleFlexionInman;
rightAnkleFlexionCappozzo = leftAnkleFlexionCappozzo;

% Scale and interpolate back moment data columns to go from being expressed
% as functions of cappozzoTimesH and cappozzoTimesB to gaitCycleTimes.  Do
% this by first affinely scaling cappozzoTimesH and cappozzoTimesB to have
% the same start and end values as gaitCycleTimes, and then
% re-interpolating the back moment data columns to have gaitCycleTimes as
% their domain using MATLAB's interp1 function.
lengthOfCappozzoTimesH = cappozzoTimesH(end) - cappozzoTimesH(1);
lengthOfCappozzoTimesB = cappozzoTimesB(end) - cappozzoTimesB(1);
lengthOfGaitCycleTimes = gaitCycleTimes(end) - gaitCycleTimes(1);
scaleFactorForCappozzoTimesH = lengthOfGaitCycleTimes / lengthOfCappozzoTimesH;
scaleFactorForCappozzoTimesB = lengthOfGaitCycleTimes / lengthOfCappozzoTimesB;
cappozzoTimesH = gaitCycleTimes(1) + ( cappozzoTimesH - cappozzoTimesH(1) ) * scaleFactorForCappozzoTimesH;
cappozzoTimesB = gaitCycleTimes(1) + ( cappozzoTimesB - cappozzoTimesB(1) ) * scaleFactorForCappozzoTimesB;
lumbarExtension = interp1(cappozzoTimesH, origLumbarExtension, gaitCycleTimes);
lumbarBending   = interp1(cappozzoTimesB, origLumbarBending  , gaitCycleTimes);
lumbarRotation  = interp1(cappozzoTimesH, origLumbarRotation , gaitCycleTimes);

% Now we have all joint moments described as discrete functions of
% gaitCycleTimes, which ranges from (0% to 100% of a gait cycle).  We then
% pass these joint moment arrays to convert_jointMomentDataToTime to get
% the joint moments for the number of gait cycles (e.g., 1.5 gait cycles)
% of the current simulation, as functions of the time array of the current
% simulation.
jointMomentData = [ lumbarExtension lumbarBending lumbarRotation ...
                    leftHipFlexionInman leftHipFlexionCappozzo leftHipFlexionCrowninshield ...
                    leftHipAdductionCrowninshield leftHipAdductionPatriarco ...
                    leftHipRotationCrowninshield leftHipRotationPatriarco ...
                    leftKneeFlexionCappozzo leftKneeFlexionInman ...
                    leftAnkleFlexionInman leftAnkleFlexionCappozzo ];
% Fill in t = 0.025 row that is empty (NaN) in all columns except the first
% 3 columns.
jointMomentData(2, 4:end) = 0.5 * (jointMomentData(1, 4:end) + jointMomentData(3, 4:end));
% The literature joint moments were scaled by the mass of the UT walking
% model, so unscale all joint moments by this mass and scale them by the
% mass of the subject whose movement is being simulated in this trial.
subjectMass = tInfo.mass; % just giving it a more descriptive name!
UTModelMass = 71.002; % in kilograms
scaleUnscaleFactor = subjectMass / UTModelMass;
jointMomentData = jointMomentData * scaleUnscaleFactor;
% Get the joint moments in the time units of this trial.
[leftTimesScaled, leftJointMomentsScaled] = convert_jointMomentDataToTime(gaitCycleTimes, jointMomentData, ...
                   ictoEvents, tInfo.analogRate, 'L', tInfo, ref_dataFormat);

lumbarExtension = leftJointMomentsScaled(:,1);
lumbarBending = leftJointMomentsScaled(:,2);
lumbarRotation = leftJointMomentsScaled(:,3);
leftHipFlexionInman = leftJointMomentsScaled(:,4);
leftHipFlexionCappozzo = leftJointMomentsScaled(:,5);
leftHipFlexionCrowninshield = leftJointMomentsScaled(:,6);
leftHipAdductionCrowninshield = leftJointMomentsScaled(:,7);
leftHipAdductionPatriarco = leftJointMomentsScaled(:,8);
leftHipRotationCrowninshield = leftJointMomentsScaled(:,9);
leftHipRotationPatriarco = leftJointMomentsScaled(:,10);
leftKneeFlexionCappozzo = leftJointMomentsScaled(:,11);
leftKneeFlexionInman = leftJointMomentsScaled(:,12);
leftAnkleFlexionInman = leftJointMomentsScaled(:,13);
leftAnkleFlexionCappozzo = leftJointMomentsScaled(:,14);

jointMomentData = [ rightHipFlexionInman rightHipFlexionCappozzo rightHipFlexionCrowninshield ...
                    rightHipAdductionCrowninshield rightHipAdductionPatriarco ...
                    rightHipRotationCrowninshield rightHipRotationPatriarco ...
                    rightKneeFlexionCappozzo rightKneeFlexionInman ...
                    rightAnkleFlexionInman rightAnkleFlexionCappozzo ];
% Fill in t = 0.025 row that is empty (NaN) in all columns.
jointMomentData(2, :) = 0.5 * (jointMomentData(1, :) + jointMomentData(3, :));
% The literature joint moments were scaled by the mass of the UT walking
% model, so unscale all joint moments by this mass and scale them by the
% mass of the subject whose movement is being simulated in this trial.
jointMomentData = jointMomentData * scaleUnscaleFactor;
% Get the joint moments in the time units of this trial.
[rightTimesScaled, rightJointMomentsScaled] = convert_jointMomentDataToTime(gaitCycleTimes, jointMomentData, ...
                   ictoEvents, tInfo.analogRate, 'R', tInfo, ref_dataFormat);

rightHipFlexionInman = rightJointMomentsScaled(:,1);
rightHipFlexionCappozzo = rightJointMomentsScaled(:,2);
rightHipFlexionCrowninshield = rightJointMomentsScaled(:,3);
rightHipAdductionCrowninshield = rightJointMomentsScaled(:,4);
rightHipAdductionPatriarco = rightJointMomentsScaled(:,5);
rightHipRotationCrowninshield = rightJointMomentsScaled(:,6);
rightHipRotationPatriarco = rightJointMomentsScaled(:,7);
rightKneeFlexionCappozzo = rightJointMomentsScaled(:,8);
rightKneeFlexionInman = rightJointMomentsScaled(:,9);
rightAnkleFlexionInman = rightJointMomentsScaled(:,10);
rightAnkleFlexionCappozzo = rightJointMomentsScaled(:,11);

% Colors for different literature joint moment curves, indicating which
% source each curve is from
colorInman = 'g';
colorCappozzo1975 = 'r';
colorCappozzo1983 = 'c';
colorCrowninshield = 'y';
colorPatriarco = 'm';

% Get labels that specify contents of figures and subplots.
% NOTE: figContents returns a structure, formatted as follows:
%           figContents(figNum).qPlotLabel{plotNum} 
%           figContents(figNum).subplotTitle{plotNum}
%           figContents(figNum).subplotAxisLabel{plotNum}
%           figContents(figNum).subplotRange{plotNum}
%           figContents(figNum).subplotTick{plotNum}    
figContents = ref_jointMomentPlotLabels;

% Specify attributes of figure windows.
nPlotRows = 5;                      
nPlotCols = 2;
nSubPlots = nPlotRows * nPlotCols;  
set(0, 'Units', 'pixels');          
scnsize = get(0, 'ScreenSize');
scnwidth = scnsize(3);
scnheight = scnsize(4);
figPos = [0.25*scnwidth, 0.05*scnheight, 0.65*scnwidth, 0.85*scnheight];
figColor = 'w';

% For each figure...
for figNum = 1:length(figHandleArray)
    figure(figHandleArray(figNum)); 
    clf
    set(gcf, 'Position', figPos, 'Color', figColor);

    % Plot literature joint moment data.
    if figNum == 2 % Hip moments
        subplot(nPlotRows, nPlotCols, 5);
        overlay_jointMomentFromLiterature(leftTimesScaled, leftHipFlexionInman, colorInman);
        overlay_jointMomentFromLiterature(leftTimesScaled, leftHipFlexionCappozzo, colorCappozzo1975);
        overlay_jointMomentFromLiterature(leftTimesScaled, leftHipFlexionCrowninshield, colorCrowninshield);
        subplot(nPlotRows, nPlotCols, 6);
        overlay_jointMomentFromLiterature(rightTimesScaled, rightHipFlexionInman, colorInman);
        overlay_jointMomentFromLiterature(rightTimesScaled, rightHipFlexionCappozzo, colorCappozzo1975);
        overlay_jointMomentFromLiterature(rightTimesScaled, rightHipFlexionCrowninshield, colorCrowninshield);
        subplot(nPlotRows, nPlotCols, 7);
        overlay_jointMomentFromLiterature(leftTimesScaled, leftHipAdductionCrowninshield, colorCrowninshield);
        overlay_jointMomentFromLiterature(leftTimesScaled, leftHipAdductionPatriarco, colorPatriarco);
        subplot(nPlotRows, nPlotCols, 8);
        overlay_jointMomentFromLiterature(rightTimesScaled, rightHipAdductionCrowninshield, colorCrowninshield);
        overlay_jointMomentFromLiterature(rightTimesScaled, rightHipAdductionPatriarco, colorPatriarco);
        subplot(nPlotRows, nPlotCols, 9);
        overlay_jointMomentFromLiterature(leftTimesScaled, leftHipRotationCrowninshield, colorCrowninshield);
        overlay_jointMomentFromLiterature(leftTimesScaled, leftHipRotationPatriarco, colorPatriarco);
        subplot(nPlotRows, nPlotCols, 10);
        overlay_jointMomentFromLiterature(rightTimesScaled, rightHipRotationCrowninshield, colorCrowninshield);
        overlay_jointMomentFromLiterature(rightTimesScaled, rightHipRotationPatriarco, colorPatriarco);
    elseif figNum == 3 % Knee and ankle moments
        subplot(nPlotRows, nPlotCols, 5);
        overlay_jointMomentFromLiterature(leftTimesScaled, leftKneeFlexionCappozzo, colorCappozzo1975);
        overlay_jointMomentFromLiterature(leftTimesScaled, leftKneeFlexionInman, colorInman);
        subplot(nPlotRows, nPlotCols, 6);
        overlay_jointMomentFromLiterature(rightTimesScaled, rightKneeFlexionCappozzo, colorCappozzo1975);
        overlay_jointMomentFromLiterature(rightTimesScaled, rightKneeFlexionInman, colorInman);
        subplot(nPlotRows, nPlotCols, 7);
        overlay_jointMomentFromLiterature(leftTimesScaled, leftAnkleFlexionCappozzo, colorCappozzo1975);
        overlay_jointMomentFromLiterature(leftTimesScaled, leftAnkleFlexionInman, colorInman);
        subplot(nPlotRows, nPlotCols, 8);
        overlay_jointMomentFromLiterature(rightTimesScaled, rightAnkleFlexionCappozzo, colorCappozzo1975);
        overlay_jointMomentFromLiterature(rightTimesScaled, rightAnkleFlexionInman, colorInman);
    elseif figNum == 4 % Low back moments
        subplot(nPlotRows, nPlotCols, 5);
        overlay_jointMomentFromLiterature(leftTimesScaled, lumbarExtension, colorCappozzo1983);
        subplot(nPlotRows, nPlotCols, 7);
        overlay_jointMomentFromLiterature(leftTimesScaled, lumbarBending, colorCappozzo1983);
        subplot(nPlotRows, nPlotCols, 9);
        overlay_jointMomentFromLiterature(leftTimesScaled, lumbarRotation, colorCappozzo1983);
    end
    
    % For each file in fnames{}...
    for fileNum = 1:length(fnames)

        % Plot vertical GRFs for reference.
        for plotIndex = 1:2
            switch plotIndex
                case 1
                    limb = 'L';
                case 2 
                    limb = 'R';
            end
            subplot(nPlotRows, nPlotCols, plotIndex);
            overlay_grftVertical(q(fileNum), ...
                figContents(figNum).qPlotLabel{plotIndex}, ...
                time{fileNum}, limb, fileNum);
        end
             
        % Plot data from SimTrack or UW-Gait Workflow.
        for plotIndex = 3:nSubPlots
            subplot(nPlotRows, nPlotCols, plotIndex);
            overlay_jointMomentsFromMot(q(fileNum), ...
                figContents(figNum).qPlotLabel{plotIndex}, ...
                time{fileNum}, fileNum);
        end
    end
    
    % Format figure.
    format_jointMomFromMot(fnames, timeRange, tInfo, ...
                figContents(figNum).subplotTitle, ...
                figContents(figNum).subplotAxisLabel, ...
                figContents(figNum).subplotRange, ...
                figContents(figNum).subplotTick);
                                
    % Add title
    titleString = sprintf('%s%s%s%3.2f%s', ...
        'Comparison of Joint Moments:  Subject ', subject, ...
        ', ', tInfo.speed, ' m/s');
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
end

% Query user.
done = printmenu(figHandleArray, [subject '_muscleFrc']);
return;
   
