function [ricFromMkr, licFromMkr] = ...
                get_icFromMkr(c, icFromGRF, fzData, tInfo, figHandle)
% Purpose:  Detects events corresponding to initial contact from the
%           forward velocity of the AJC.
%
%           NOTE:  The velocity threshold for detecting IC events is 
%           initially defined from 'known' IC events determined from the 
%           vertical GRF data; however, the user is allowed to re-adjust 
%           this threshold interactively.
%
% Input:    c is a structure returned from read_c3DFile()
%           icFromGRF(fpHitNum) is an array of IC events, in analog frames,
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
% Output:   ricFromMkr and licFromMkr are arrays of IC events, 
%               corresponding to the R and L limbs, respectively,
%               converted to analog frames.
%
% Called Functions:  
%       get_ddt(x, t)
%       get_defThreshold(dX, dt, limb, eFromGRF, c, tInfo)
%       detect_icTimes(dX, dt, threshold)
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

% Get lists of labels corresponding to the R and L joint centers in 
% structure c.
for jntcenterIndex = 1:length(c.jntcenters.R)
    RJCLabels{jntcenterIndex} = c.jntcenters.R(jntcenterIndex).label;
    LJCLabels{jntcenterIndex} = c.jntcenters.L(jntcenterIndex).label;
end

% Get fore-aft trajectories of the R and L AJCs. 
% If subject walked in the -x direction, flip sign of coordinate data.
% NOTE:  The C3D Server returns cell arrays;
%        these arrays are converted to numeric arrays.
RajcIndex = strmatch('RTIO', RJCLabels);
LajcIndex = strmatch('LTIO', LJCLabels);
for vframeNum = 1:nVideoFrames
    RajcX(vframeNum) = c.jntcenters.R(RajcIndex).data{vframeNum, 1};   
    LajcX(vframeNum) = c.jntcenters.L(LajcIndex).data{vframeNum, 1}; 
end
if tInfo.FP{1} > tInfo.FP{2}  
    RajcX = -1*RajcX;
    LajcX = -1*LajcX;
end

% Get fore-aft velocities of the R and L AJCs. 
[dRajcX, tdRajcX] = get_ddt(RajcX, vTime);
[dLajcX, tdLajcX] = get_ddt(LajcX, vTime);

% Get default velocity thresholds for detecting contact.
ricThreshold = get_defThreshold(dRajcX, tdRajcX, 'R', icFromGRF, c, tInfo);
licThreshold = get_defThreshold(dLajcX, tdLajcX, 'L', icFromGRF, c, tInfo);

% Define query and options for user.
query = 'Adjust Velocity Threshold?';       
opt1 = 'threshold ++';
opt2 = 'threshold --';
opt3 = 'print figure';
opt4 = 'done';

% Iteratively detect and plot R IC events until satisfied w/ threshold.
done = 0;
while ~done   
    ricTimes = detect_icTimes(dRajcX, tdRajcX, ricThreshold);
    plot_ictoFromMkr(fzData, aTime, RajcX, vTime, dRajcX, tdRajcX, ...
              icFromGRF, ricTimes, ricThreshold, 'R', c, tInfo, figHandle);
    titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
        'Events from Markers:  Subject ', char(c.subject), '-',  ...
        tInfo.trial, ', Speed ', tInfo.speed, ' m/s, RIGHT IC'); 
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
    userInput = menu(query, opt1, opt2, opt3, opt4);
    switch userInput
        case 1
            ricThreshold = ricThreshold + 0.01*max(dRajcX);
        case 2
            ricThreshold = ricThreshold - 0.01*max(dRajcX);
        case 3
            orient(figHandle, 'tall');  
            printCommand = ['print -f', num2str(figHandle), ' -r600'];
            eval(printCommand);
            done = 1;
        case 4
            done = 1;
    end
end

% Iteratively detect and plot L IC events until satisfied w/ threshold.
done = 0;
while ~done    
    licTimes = detect_icTimes(dLajcX, tdLajcX, licThreshold);
    plot_ictoFromMkr(fzData, aTime, LajcX, vTime, dLajcX, tdLajcX, ...
              icFromGRF, licTimes, licThreshold, 'L', c, tInfo, figHandle);
    titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
        'Events from Markers:  Subject ', char(c.subject), '-',  ...
        tInfo.trial, ', Speed ', tInfo.speed, ' m/s, LEFT IC'); 
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
    userInput = menu(query, opt1, opt2, opt3, opt4);
    switch userInput
        case 1
            licThreshold = licThreshold + 0.01*max(dLajcX);
        case 2
            licThreshold = licThreshold - 0.01*max(dLajcX);
        case 3
            orient(figHandle, 'tall');  
            printCommand = ['print -f', num2str(figHandle), ' -r600'];
            eval(printCommand);
            done = 1;
        case 4
            done = 1;
    end
end

% Convert IC events from time to analog frames.
ricFromMkr = ricTimes * c.analog.rate;
licFromMkr = licTimes * c.analog.rate;
return;
