function corrected_kAxisAngles = ...
    evaluate_c3dkAxisMalalignment(cSim, tInfo, ictoEvents, figHandleArray)
% Purpose:  Calls functions to:
%           (1) extract joint angle data from cSim needed to estimate 
%               and correct for malalignment of the knee flex/ext axes.
%           (2) estimate the degree of axes malalignment based on the 
%               'measured' knee angles extracted from cSim
%           (3) plot the 'corrected' knee flex/ext, knee var/val, 
%               knee rotation, and hip rotation angles
%           (4) query the user to decide whether or not these offsets
%               should be used.
%
% Input:    cSim is a structure returned from extract_c3dSimData()
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.trial - trial number to analyze ('character array')
%               *.speed - average walking speed during trial in m/s
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%           ictoEvents is a structure describing the timing of events
%               associated with each FP hit, in analog frames:
%               *(fpHitNum).ic       - initial contact
%               *(fpHitNum).oto      - opposite toe off
%               *(fpHitNum).oic      - opposite initial contact
%               *(fpHitNum).to       - toe off
%               *(fpHitNum).icNext   - initial contact
%           figHandleArray is a 1x2 array corresponding to the numbers of
%               the figure windows for plotting the R and L angles.
%
% Output:  corrected_kAxisAngles returns a structure containing the
%          'corrected' knee flex/ext, knee var/val, knee rotation,
%           and hip rotation angles, in this format:
%               *.R     .kf(nVideoFrames of 'simulateable' segment)
%               *.L     .kv(")
%                       .kr(")
%                       .hr(")
%                       .kAxisOffset
%           NOTE:  corrected_kAxisAngles are in the Gillette lab CS.
%
% Called Functions:
%       read_gcdMean(fname)
%       convert_cycleToTime(cycle, tInfo, ictoEvents, analogRate)
%       get_kAxisMeasuredAngles(cSim)
%       get_kAxisOffset(measuredAngles, limb)
%       get_kAxisCorrectedAngles(measuredAngles, offset, limb)
%       create_c3dkAxisFig(measuredAngles, correctedAngles, g, ...
%                                                vTime, gTimes, figHandle)
%       Suptitle(titleString)
%
% ASA, 10-05


% Read averaged data from control subjects, for reference.
g = read_gcdMean('gilletteMeanSD.GCD');

% Get arrays that convert %gait cycle values to time values corresponding
% to 'simulateable' segment for R and L limbs.
cycTimes = ...
    convert_cycleToTime(g.cycle, tInfo, ictoEvents, cSim.analog.rate);

% Define array of time values corresponding to the 'simulateable' data.
% Set t = 0 at IC of the 1st FP strike; note that the data starts at OTO, 
% which occurs at cSim.tDS seconds later.
vFrames = 1:1:cSim.video.nframes;
vTime = vFrames/cSim.video.rate + cSim.tDS;

% Extract measured joint angle data from 'simulateable' segment needed to 
% estimate and correct for malalignment of the knee flex/ext axis.
measured_kAxisAngles = get_kAxisMeasuredAngles(cSim);

% Estimate the knee axis offsets.
kAxisOffset.R = get_kAxisOffset(measured_kAxisAngles.R, 'R');
kAxisOffset.L = get_kAxisOffset(measured_kAxisAngles.L, 'L');


% Plot the measured and corrected angles for each limb.  
% NOTE:  The user is allowed to tweak kAxisOffset, if desired, 
%        and decide whether the measured or corrected data should be used.

%%% CORRECTION FOR RIGHT LIMB
done = 0;
while ~done

    % Compute the 'corrected' hip and knee angles.
    corrected_kAxisAngles.R = ...
      get_kAxisCorrectedAngles(measured_kAxisAngles.R, kAxisOffset.R, 'R');

    % Plot measured and corrected angles and query user.
    create_c3dkAxisFig(measured_kAxisAngles.R, corrected_kAxisAngles.R, ...
                            g, vTime, cycTimes.R, figHandleArray(1));
    titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
        'Correction for kAxis Malalignment, R Limb:  Subject ', ...
        char(cSim.subject), '-', tInfo.trial, ', Speed ', ...
        tInfo.speed, ' m/s');
    Suptitle(titleString);    % MATLAB m-file for adding "super title"

    % Query user.
    query = 'Adjust kAxis offset, or select solution?';
    opt1 = 'offset ++';
    opt2 = 'offset --';
    opt3 = 'print';
    opt4 = 'done: use measured';
    opt5 = 'done: use corrected';
    userInput = menu(query, opt1, opt2, opt3, opt4, opt5);
    switch userInput
        case 1
            kAxisOffset.R = kAxisOffset.R + 1;
        case 2
            kAxisOffset.R = kAxisOffset.R - 1;
        case 3
            orient(figHandleArray(1), 'landscape');  
            printCommand = ['print -f', num2str(figHandleArray(1)), ' -r600'];
            eval(printCommand);
        case 4
            corrected_kAxisAngles.R = measured_kAxisAngles.R;
            done = 1;   
        case 5
            done = 1;
    end
end

 %%% CORRECTION FOR LEFT LIMB
done = 0;
while ~done

    % Compute the 'corrected' hip and knee angles.
    corrected_kAxisAngles.L = ...
      get_kAxisCorrectedAngles(measured_kAxisAngles.L, kAxisOffset.L, 'L');

    % Plot measured and corrected angles and query user.
    create_c3dkAxisFig(measured_kAxisAngles.L, corrected_kAxisAngles.L, ...
                            g, vTime, cycTimes.L, figHandleArray(2));
    titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
        'Correction for kAxis Malalignment, L Limb:  Subject ', ...
        char(cSim.subject), '-', tInfo.trial, ', Speed ', ...
        tInfo.speed, ' m/s');
    Suptitle(titleString);    % MATLAB m-file for adding "super title"

    % Query user.
    query = 'Tweak kAxis offset, or select solution?';
    opt1 = 'offset ++';
    opt2 = 'offset --';
    opt3 = 'print';
    opt4 = 'done: use measured';
    opt5 = 'done: use corrected';
    userInput = menu(query, opt1, opt2, opt3, opt4, opt5);
    switch userInput
        case 1
            kAxisOffset.L = kAxisOffset.L + 1;
        case 2
            kAxisOffset.L = kAxisOffset.L - 1;
        case 3
            orient(figHandleArray(2), 'landscape');  
            printCommand = ['print -f', num2str(figHandleArray(2)), ' -r600'];
            eval(printCommand);
        case 4
            corrected_kAxisAngles.L = measured_kAxisAngles.L;
            done = 1;   
        case 5
            done = 1;
    end
end
return;  
