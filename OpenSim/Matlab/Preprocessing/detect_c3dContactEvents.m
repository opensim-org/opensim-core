function [] = detect_c3dContactEvents(subject, tInfo, ref_dataFormat)
% Purpose:  Reads the C3D file of a Gillette control subject, estimates 
%           event times corresponding to initial contact, foot-flat, 
%           heel-off, and toe-off over a 'simulateable' segment of data,
%           and writes these times to the screen.
%     
% Input:    subject is a 6-digit subject ID ('character array')
%           tInfo is a structure containing the following 'trial info':
%               *.trial - trial number to analyze ('character array')
%               *.mass  - mass of subject in kg
%               *.speed - average walking speed during trial in m/s
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%               *.ictoMatrix - matrix of ic, oto, oic, to, & icNext events, 
%                               in analog frames, one row per FP hit 
%
% Notes:    1. This code is interactive; the user is queried to adjust the
%              threshold used for event detection.
%           2. Times are referenced to IC of the 1st FP strike, which is
%              defined as t = 0.
%           3. Although the AJC and TOE marker data are plotted vs. 
%              analog time over the entire trial for reference, only events 
%              corresponding to a 'simulateable' segment, defined by the 
%              ictoEvents, are detected.  
%
% Called Functions:
%       read_c3dFile(fname)
%       get_ictoEvents(c, tInfo, figHandle)
%       get_vertGRF(c, tInfo)
%       get_ffFromMkr(c, ricTimes, licTimes, fzR, fzL, tInfo, figHandle)
%       get_hoFromMkr(c, rtoTimes, ltoTimes, fzR, fzL, tInfo, figHandle) 
%
% ASA, 9-05


% Display status on screen.
status = sprintf('\n\n\n%s%s%s%s%s%3.2f%s', ...
          'Detecting Foot/Ground Contact Times for Subject ', subject, ...
          ', Trial ', tInfo.trial, ...
          ', Speed ', tInfo.speed, ' m/s');
disp(status);

% Read data from C3D file.
fname = [subject, ' ', tInfo.trial, '.c3d'];
c = read_c3dFile(fname); 

% Get analog frames corresponding to IC and TO events; 
% if icto data have not been previously stored, call get_ictoEvents();
% if icto data have been stored, convert matrix to structure.
if isempty(tInfo.ictoMatrix)            
    figHandle = 1;
    ictoEvents = get_ictoEvents(c, tInfo, figHandle);
else                                    
    for fpHitNum = 1:length(tInfo.FP)     
        ictoEvents(fpHitNum).ic  = tInfo.ictoMatrix(fpHitNum, 1);
        ictoEvents(fpHitNum).oto = tInfo.ictoMatrix(fpHitNum, 2);
        ictoEvents(fpHitNum).oic = tInfo.ictoMatrix(fpHitNum, 3);
        ictoEvents(fpHitNum).to  = tInfo.ictoMatrix(fpHitNum, 4);
        ictoEvents(fpHitNum).icNext = tInfo.ictoMatrix(fpHitNum, 5);
    end
end

% Convert IC and TO events, from ictoEvents, to analog time.
rIndices = strmatch('R', tInfo.limb);
ricTimes = [ictoEvents(rIndices).ic ictoEvents(max(rIndices)).icNext] ...
                ./ c.analog.rate;
rtoTimes = [ictoEvents(rIndices).to] ./c.analog.rate;
            
lIndices = strmatch('L', tInfo.limb);
licTimes = [ictoEvents(lIndices).ic ictoEvents(max(lIndices)).icNext] ...
                ./ c.analog.rate;
ltoTimes = [ictoEvents(lIndices).to] ./ c.analog.rate;
               
% Get vertical GRFs (over entire trial), for reference.
[fzR, fzL] = get_vertGRF(c, tInfo);

% Get FF and HO events.
figHandle = 1;
[rffTimes, lffTimes] = ...
    get_ffFromMkr(c, ricTimes, licTimes, fzR, fzL, tInfo, figHandle); 
[rhoTimes, lhoTimes] = ...
    get_hoFromMkr(c, rtoTimes, ltoTimes, fzR, fzL, tInfo, figHandle); 

% Label events and store in structure for sorting.
for i = 1:length(ricTimes)
    eLabels{i} = 'R IC';
    eTimes(i) = ricTimes(i);
end
nEvents = length(eLabels);
for i = 1:length(licTimes)
    eLabels{nEvents + i} = 'L IC';
    eTimes(nEvents + i) = licTimes(i);
end
nEvents = length(eLabels);
for i = 1:length(rffTimes)
    eLabels{nEvents + i} = 'R FF';
    eTimes(nEvents + i) = rffTimes(i);
end
nEvents = length(eLabels);
for i = 1:length(lffTimes)
    eLabels{nEvents + i} = 'L FF';
    eTimes(nEvents + i) = lffTimes(i);
end
nEvents = length(eLabels);
for i = 1:length(rhoTimes)
    eLabels{nEvents + i} = 'R HO';
    eTimes(nEvents + i) = rhoTimes(i);
end
nEvents = length(eLabels);
for i = 1:length(lhoTimes)
    eLabels{nEvents + i} = 'L HO';
    eTimes(nEvents + i) = lhoTimes(i);
end
nEvents = length(eLabels);
for i = 1:length(rtoTimes)
    eLabels{nEvents + i} = 'R TO';
    eTimes(nEvents + i) = rtoTimes(i);
end
nEvents = length(eLabels);
for i = 1:length(ltoTimes)
    eLabels{nEvents + i} = 'L TO';
    eTimes(nEvents + i) = ltoTimes(i);
end
nEvents = length(eLabels);

% Sort events by time.
[sorted_times, sortIndex] = sort(eTimes);
for eventNum = 1:nEvents
    eMkrs(eventNum).label  = eLabels{sortIndex(eventNum)};
    eMkrs(eventNum).time  = sorted_times(eventNum);
end

% Convert from analog time to 'simulateable' time.
if ref_dataFormat.tzeroAtFirstIC
	tZeroInFrames = ictoEvents(1).ic; 
else
	% EG: the gcd values lined up with my computed motions better
	%     if I use 1 here rather than the first IC time
	tZeroInFrames = 1;
end
tZero = tZeroInFrames/c.analog.rate;
for eventNum = 1:nEvents
    eMkrs(eventNum).time = eMkrs(eventNum).time - tZero;
end

% Write sorted event data to screen.
tableTitle = sprintf('\n%10s\t\t%s', ...
                     'Events:', 'Time from IC of 1st FP Hit:');
disp(tableTitle);
for eventNum = 1:length(eMkrs)
    eMkrs(eventNum).table = sprintf('%10s\t\t%6.4f', ...
                                eMkrs(eventNum).label,  ...
                                eMkrs(eventNum).time);
    disp(eMkrs(eventNum).table);    
end
return;
