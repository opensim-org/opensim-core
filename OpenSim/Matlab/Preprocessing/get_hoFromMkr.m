function [rhoTimes, lhoTimes] = get_hoFromMkr(c, rtoTimes, ltoTimes, ...
                                    fzR, fzL, tInfo, figHandle)
% Purpose:  Detects events corresponding to heel off from the
%           vertical position of the AJC.
%
%           NOTE:  The position threshold for detecting HO events is 
%           initially defined from the median position of the AJC; 
%           however, the user is allowed to re-adjust this threshold 
%           interactively.
%
% Input:    c is a structure returned from read_c3dFile()
%           rtoTimes and ltoTimes are arrays of TO events, in units 
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
% Output:   rhoTimes and lhoTimes are arrays of HO events, corresponding 
%               to the R and L limbs, respectively, in units corresponding 
%               to analog time
%
% Called Functions:  
%       get_ddt(x, t)
%       detect_hoTimes(Z, t, toTimes, threshold)
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

% Get lists of labels corresponding to R and L joint centers
% in structure c.
for jntcenterIndex = 1:length(c.jntcenters.R)
    RJCLabels{jntcenterIndex} = c.jntcenters.R(jntcenterIndex).label;
    LJCLabels{jntcenterIndex} = c.jntcenters.L(jntcenterIndex).label;
end

% Get vertical trajectories of the R and L AJC. 
% NOTE:  The C3D Server returns cell arrays;
%        these arrays are converted to numeric arrays.
RajcIndex = strmatch('RTIO', RJCLabels);
LajcIndex = strmatch('LTIO', LJCLabels);
for vframeNum = 1:nVideoFrames
    RajcZ(vframeNum) = c.jntcenters.R(RajcIndex).data{vframeNum, 3};   
    LajcZ(vframeNum) = c.jntcenters.L(LajcIndex).data{vframeNum, 3}; 
end

% Get vertical velocities of the R and L TOE.
[dRajcZ, tdRajcZ] = get_ddt(RajcZ, vTime);
[dLajcZ, tdLajcZ] = get_ddt(LajcZ, vTime);

% Get default position thresholds for detecting contact;
%   assume median vertical position of the AJC is representative.
rhoThreshold = median(RajcZ);
lhoThreshold = median(LajcZ);

% Define query and options for user.
query = 'Adjust Position Threshold?';       
opt1 = 'threshold ++';
opt2 = 'threshold --';
opt3 = 'print figure';
opt4 = 'done';

% Iteratively detect and plot R HO events until satisfied w/ threshold.
done = 0;
while ~done   
    rhoTimes = detect_hoTimes(RajcZ, vTime, rtoTimes, rhoThreshold);
    plot_ffhoFromMkr(fzR, aTime, RajcZ, vTime, dRajcZ, tdRajcZ, ...
                            rhoTimes, rhoThreshold, 'R', figHandle);
    titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
        'Events from Markers:  Subject ', char(c.subject), '-',  ...
        tInfo.trial, ', Speed ', tInfo.speed, ' m/s, RIGHT HO'); 
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
    userInput = menu(query, opt1, opt2, opt3, opt4);
    switch userInput
        case 1
            rhoThreshold = rhoThreshold + 0.01*max(RajcZ);
        case 2
            rhoThreshold = rhoThreshold - 0.01*max(RajcZ);
        case 3
            orient(figHandle, 'tall');  
            printCommand = ['print -f', num2str(figHandle), ' -r600'];
            eval(printCommand);
            done = 1;
        case 4
            done = 1;
    end
end

% Iteratively detect and plot L HO events until satisfied w/ threshold.
done = 0;
while ~done    
    lhoTimes = detect_hoTimes(LajcZ, vTime, ltoTimes, lhoThreshold);
    plot_ffhoFromMkr(fzL, aTime, LajcZ, vTime, dLajcZ, tdLajcZ, ...
                            lhoTimes, lhoThreshold, 'L', figHandle);
    titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
        'Events from Markers:  Subject ', char(c.subject), '-',  ...
        tInfo.trial, ', Speed ', tInfo.speed, ' m/s, LEFT HO'); 
    Suptitle(titleString);      % MATLAB m-file for adding "super title"
    userInput = menu(query, opt1, opt2, opt3, opt4);
    switch userInput
        case 1
            lhoThreshold = lhoThreshold + 0.02*max(LajcZ);
        case 2
            lhoThreshold = lhoThreshold - 0.02*max(LajcZ);
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
