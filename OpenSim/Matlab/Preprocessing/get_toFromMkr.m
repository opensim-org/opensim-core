function [rtoFromMkr, ltoFromMkr] = ...
                get_toFromMkr(c, toFromGRF, fzData, tInfo, figHandle)
% Purpose:  Detects events corresponding to initial contact from the
%           forward velocity of the TOE.
%
%           NOTE:  The velocity threshold for detecting TO events is 
%           initially defined from 'known' TO events determined from the 
%           vertical GRF data; however, the user is allowed to re-adjust 
%           this threshold interactively.
%
% Input:    c is a structure returned from read_c3DFile()
%           toFromGRF(fpHitNum) is an array of TO events, in analog frames,
%               one per FP hit, in the order specified by tInfo.FP.
%           fzData(aFrameNum, fpHitNum) is a matrix of vertical GRF data.
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.trial - trial number to analyze ('character array')
%               *.speed - average walking speed during trial in m/s
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%           figHandle specifies the number of the figure window,
%               used to display events detected from marker data.
%
% Output:   rtoFromMkr and ltoFromMkr are arrays of TO events, 
%               corresponding to the R and L limbs, respectively,
%               converted to analog frames.
%
% Called Functions:  
%       get_ddt(x, t)
%       get_defThreshold(dX, dt, limb, eFromGRF, c, tInfo)
%       detect_toTimes(dX, dt, threshold)
%       plot_ictoFromMkr(fzData, aTime, X, vTime, dX, dt, ...
%               eFromGRF, eTimes, threshold, limb, c, tInfo, figHandle)
%       Suptitle(titleString)
%
% ASA, 9-05


% Get relevant dimensions and arrays.
nVideoFrames = c.video.nframes;
vFrames = 1:1:nVideoFrames;
vTime = vFrames/c.video.rate;
nAnalogFrames = c.video.nframes * c.analog.ratio;
aFrames = 1:1:nAnalogFrames;
aTime = aFrames/c.analog.rate;

% Get lists of labels corresponding to the R and L marker data in 
% structure c.
for markerIndex = 1:length(c.markers)
    markerLabels{markerIndex} = c.markers(markerIndex).label;
end

% Get fore-aft trajectories of the R and L TOE. 
% If subject walked in the -x direction, flip sign of coordinate data.
% NOTE:  The C3D Server returns cell arrays;
%        these arrays are converted to numeric arrays.
RtoeIndex = strmatch('RTOE', markerLabels);
LtoeIndex = strmatch('LTOE', markerLabels);
for vframeNum = 1:nVideoFrames
    RtoeX(vframeNum) = c.markers(RtoeIndex).data{vframeNum, 1};
    LtoeX(vframeNum) = c.markers(LtoeIndex).data{vframeNum, 1};
end
if tInfo.FP{1} > tInfo.FP{2}  
    RtoeX = -1*RtoeX;
    LtoeX = -1*LtoeX;
end

% Get fore-aft velocities of the R and L TOE.
[dRtoeX, tdRtoeX] = get_ddt(RtoeX, vTime);
[dLtoeX, tdLtoeX] = get_ddt(LtoeX, vTime);

% Get default velocity thresholds for detecting contact.
rtoThreshold = get_defThreshold(dRtoeX, tdRtoeX, 'R', toFromGRF, c, tInfo);
ltoThreshold = get_defThreshold(dLtoeX, tdLtoeX, 'L', toFromGRF, c, tInfo);

% Define query and options for user.
query = 'Adjust Velocity Threshold?';       
opt1 = 'threshold ++';
opt2 = 'threshold --';
opt3 = 'print figure';
opt4 = 'done';

% Iteratively detect and plot R TO events until satisfied w/ threshold.
done = 0;
while ~done   
    rtoTimes = detect_toTimes(dRtoeX, tdRtoeX, rtoThreshold);
    plot_ictoFromMkr(fzData, aTime, RtoeX, vTime, dRtoeX, tdRtoeX, ...
              toFromGRF, rtoTimes, rtoThreshold, 'R', c, tInfo, figHandle);
    titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
        'Events from Markers:  Subject ', char(c.subject), '-',  ...
        tInfo.trial, ', Speed ', tInfo.speed, ' m/s, RIGHT TO'); 
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
    userInput = menu(query, opt1, opt2, opt3, opt4);
    switch userInput
        case 1
            rtoThreshold = rtoThreshold + 0.01*max(dRtoeX);
        case 2
            rtoThreshold = rtoThreshold - 0.01*max(dRtoeX);
        case 3
            orient(figHandle, 'tall');  
            printCommand = ['print -f', num2str(figHandle), ' -r600'];
            eval(printCommand);
            done = 1;
        case 4
            done = 1;
    end
end

% Iteratively detect and plot L TO events until satisfied w/ threshold.
done = 0;
while ~done    
    ltoTimes = detect_toTimes(dLtoeX, tdLtoeX, ltoThreshold);
    plot_ictoFromMkr(fzData, aTime, LtoeX, vTime, dLtoeX, tdLtoeX, ...
              toFromGRF, ltoTimes, ltoThreshold, 'L', c, tInfo, figHandle);
    titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
        'Events from Markers:  Subject ', char(c.subject), '-',  ...
        tInfo.trial, ', Speed ', tInfo.speed, ' m/s, LEFT TO'); 
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
    userInput = menu(query, opt1, opt2, opt3, opt4);
    switch userInput
        case 1
            ltoThreshold = ltoThreshold + 0.01*max(dLtoeX);
        case 2
            ltoThreshold = ltoThreshold - 0.01*max(dLtoeX);
        case 3
            orient(figHandle, 'tall');  
            printCommand = ['print -f', num2str(figHandle), ' -r600'];
            eval(printCommand);
            done = 1;
        case 4
            done = 1;
    end
end

% Put TO times in ascending order.
rtoTimes = sort(rtoTimes);
ltoTimes = sort(ltoTimes);

% Convert TO events from time to analog frames.
rtoFromMkr = rtoTimes * c.analog.rate;
ltoFromMkr = ltoTimes * c.analog.rate;
return;
