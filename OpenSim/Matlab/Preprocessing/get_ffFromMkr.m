function [rffTimes, lffTimes] = get_ffFromMkr(c, ricTimes, licTimes, ...
                                        fzR, fzL, tInfo, figHandle)
% Purpose:  Detects events corresponding to foot flat from the
%           vertical position of the toe.
%
%           NOTE:  The position threshold for detecting FF events is 
%           initially defined from the median position of the toe; 
%           however, the user is allowed to re-adjust this threshold 
%           interactively.
%
% Input:    c is a structure returned from read_c3dFile()
%           ricTimes and licTimes are arrays of IC events, in units 
%               corresponding to analog time
%           fzR and fzL are matrices(aFrameNum, cycleNum) of vertical GRFs
%               for the specified limb, sampled at the analog rate, where
%               each column corresponds to one FP strike.
%           tInfo is a structure containing the following 'trial info':
%               *.trial - trial number to analyze ('character array')
%               *.mass  - mass of subject in kg
%               *.speed - average walking speed during trial in m/s
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%               *.ictoMatrix - matrix of ic, oto, oic, to, & icNext events, 
%                               in analog frames, one row per FP hit 
%           figHandle specifies the number of the figure window,
%               used to display events detected from marker data.
%
% Output:   rffTimes and lffTimes are arrays of FF events, corresponding 
%               to the R and L limbs, respectively, in units corresponding
%               to analog time
%
% Called Functions:  
%       get_ddt(x, t)
%       detect_ffTimes(Z, t, icTimes, threshold)
%       plot_ffhoFromMkr(fz, aTime, Z, vTime, dZ, dt, ...
%                               eTimes, threshold, limb, figHandle)
%       Suptitle(titleString)
%
% ASA, 9-05


% Get arrays of time values corresponding to analog and video frames.
nAnalogFrames = c.video.nframes * c.analog.ratio;
aTime = (1:1:nAnalogFrames)/c.analog.rate;
nVideoFrames = c.video.nframes;
vTime = (1:1:nVideoFrames)/c.video.rate;

% Get lists of labels corresponding to the R and L marker data in 
% structure c.
for markerIndex = 1:length(c.markers)
    markerLabels{markerIndex} = c.markers(markerIndex).label;
end

% Get vertical trajectories of the R and L TOE. 
% NOTE:  The C3D Server returns cell arrays;
%        these arrays are converted to numeric arrays.
RtoeIndex = strmatch('RTOE', markerLabels);
LtoeIndex = strmatch('LTOE', markerLabels);
for vframeNum = 1:nVideoFrames
    RtoeZ(vframeNum) = c.markers(RtoeIndex).data{vframeNum, 3};
    LtoeZ(vframeNum) = c.markers(LtoeIndex).data{vframeNum, 3};
end

% Get vertical velocities of the R and L TOE.
[dRtoeZ, tdRtoeZ] = get_ddt(RtoeZ, vTime);
[dLtoeZ, tdLtoeZ] = get_ddt(LtoeZ, vTime);

% Get default position thresholds for detecting contact;
%   assume median vertical position of the toe is representative.
rffThreshold = median(RtoeZ);
lffThreshold = median(LtoeZ);

% Define query and options for user.
query = 'Adjust Position Threshold?';       
opt1 = 'threshold ++';
opt2 = 'threshold --';
opt3 = 'print figure';
opt4 = 'done';

% Iteratively detect and plot R FF events until satisfied w/ threshold.
done = 0;
while ~done   
    rffTimes = detect_ffTimes(RtoeZ, vTime, ricTimes, rffThreshold);
    plot_ffhoFromMkr(fzR, aTime, RtoeZ, vTime, dRtoeZ, tdRtoeZ, ...
                            rffTimes, rffThreshold, 'R', figHandle);
    titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
        'Events from Markers:  Subject ', char(c.subject), '-',  ...
        tInfo.trial, ', Speed ', tInfo.speed, ' m/s, RIGHT FF'); 
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
    userInput = menu(query, opt1, opt2, opt3, opt4);
    switch userInput
        case 1
            rffThreshold = rffThreshold + 0.01*max(RtoeZ);
        case 2
            rffThreshold = rffThreshold - 0.01*max(RtoeZ);
        case 3
            orient(figHandle, 'tall');  
            printCommand = ['print -f', num2str(figHandle), ' -r600'];
            eval(printCommand);
            done = 1;
        case 4
            done = 1;
    end
end

% Iteratively detect and plot L FF events until satisfied w/ threshold.
done = 0;
while ~done    
    lffTimes = detect_ffTimes(LtoeZ, vTime, licTimes, lffThreshold);
    plot_ffhoFromMkr(fzL, aTime, LtoeZ, vTime, dLtoeZ, tdLtoeZ, ...
                            lffTimes, lffThreshold, 'L', figHandle);
    titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
        'Events from Markers:  Subject ', char(c.subject), '-',  ...
        tInfo.trial, ', Speed ', tInfo.speed, ' m/s, LEFT FF'); 
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
    userInput = menu(query, opt1, opt2, opt3, opt4);
    switch userInput
        case 1
            lffThreshold = lffThreshold + 0.02*max(LtoeZ);
        case 2
            lffThreshold = lffThreshold - 0.02*max(LtoeZ);
        case 3
            orient(figHandle, 'tall');  
            printCommand = ['print -f', num2str(figHandle), ' -r600'];
            eval(printCommand);
            done = 1;
        case 4
            done = 1;
    end
end
return;
