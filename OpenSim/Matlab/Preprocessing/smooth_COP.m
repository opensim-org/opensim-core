function COP_smoothed = smooth_COP(GRFTz_byLimb, cSim, tInfo, figHandleArray)
% Purpose:  Plots GRFTz data vs time and interactively eliminates 
%           discontinuities in the COP trajectories between FP hits
%           (when the foot is out of contact with the ground);
%           this allows the COP trajectories to be fit with a spline 
%           in the simulation workflow.   
%
% Input:    GRFTz_byLimb is a structure with the following format:
%               *.R         .Fx(nAnalogFrames)
%               *.L         .Fy(nAnalogFrames)
%                           .Fz(nAnalogFrames)
%                           .Tz(nAnalogFrames)
%                           .COPx(nAnalogFrames)
%                           .COPy(nAnalogFrames)
%                           .startIndex(nCycles) - analog frame numbers 
%                               indicating start of COP non-zero data
%                           .stopIndex(nCycles) - analog frame numbers 
%                               indicating end of COP non-zero data
%           cSim is a structure read from extract_c3dSimData()
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.FP    - FP #s in  the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%           figHandleArray is an array specifying the figure window numbers
%
% Output:   COP_smoothed returns a structure with the following format:
%               *.R         .COPx(nAnalogFrames)
%               *.L         .COPy(nAnalogFrames)
%
% Notes:    1. This code is interactive; the user is queried to adjust the
%              tolerance parameter used for smoothing end effects.
%           2. Times are referenced to IC of the 1st FP strike, which
%              is defined as t = 0.
%
% Called Functions:
%   mirror_array(inArray, mirrorIndex, gapIndex)
%   interpolate_array(inArray, gapStartIndex, gapStopIndex) 
%   plot_GRFTzByLimb(GRFTzData, smCOP, aTime, gapTol, tInfo, figHandle)
%   Suptitle(titleString)
%
% ASA, 10-05


% Extract data based on FP hit order.
% Define limb1 = limb corresponding to 1st FP hit.
% Define limb2 = limb corresponding to 2nd FP hit.
if strcmpi(tInfo.limb(1), 'R')
    limb1.COPx = GRFTz_byLimb.R.COPx;
    limb1.COPy = GRFTz_byLimb.R.COPy;
    limb1.startIndex = GRFTz_byLimb.R.startIndex;
    limb1.stopIndex = GRFTz_byLimb.R.stopIndex;    
    limb2.COPx = GRFTz_byLimb.L.COPx;
    limb2.COPy = GRFTz_byLimb.L.COPy;
    limb2.startIndex = GRFTz_byLimb.L.startIndex;
    limb2.stopIndex = GRFTz_byLimb.L.stopIndex;
elseif strcmpi(tInfo.limb(1), 'L')
    limb1.COPx = GRFTz_byLimb.L.COPx;
    limb1.COPy = GRFTz_byLimb.L.COPy;
    limb1.startIndex = GRFTz_byLimb.L.startIndex;
    limb1.stopIndex = GRFTz_byLimb.L.stopIndex;    
    limb2.COPx = GRFTz_byLimb.R.COPx;
    limb2.COPy = GRFTz_byLimb.R.COPy;
    limb2.startIndex = GRFTz_byLimb.R.startIndex;
    limb2.stopIndex = GRFTz_byLimb.R.stopIndex;
end    
    
% Get analog frame numbers corresponding to initial and final times of
% 'simulateable segment'.
tiIndex = 1;
tfIndex = length(GRFTz_byLimb.R.Fx);

% Define array of time values corresponding to the 'simulateable' data.
% Set t = 0 at IC of the 1st FP strike; note that the data starts at OTO, 
% which occurs at cSim.tDS seconds later.
nAnalogFrames = cSim.video.nframes * cSim.analog.ratio;
aFrames = 1:1:nAnalogFrames;
aTime = aFrames/cSim.analog.rate + cSim.tDS;

% Set default values for tolerances used to tweak location of mirrorIndex.
% NOTE:  To avoid mirroring about a point corrupted by end effects in the
%        COP data, the user is allowed to interactively tweak the location 
%        of mirrorIndex by the number of analog frames specified by tol.
tol1 = 25;
tol2 = 25;

% Note number of FP hits.
nHits = length(tInfo.FP);

% Smooth and plot data for limb1 until user is satisfied.
done = 0;
while ~done
    
    % Smooth COP data, depending on number of FP hits.
    switch nHits    
        case 2  
            % For limb1, reflect about stopIndex to final time.
            sm1.COPx = ...
                mirror_array(limb1.COPx, limb1.stopIndex(1)-tol1, tfIndex); 
            sm1.COPy = ...
                mirror_array(limb1.COPy, limb1.stopIndex(1)-tol1, tfIndex);

        case 3  
            % For limb1, interpolate between end of 1st hit & start of 2nd.
            sm1.COPx = interpolate_array(limb1.COPx, ...
                        limb1.stopIndex(1)-tol1, limb1.startIndex(2)+tol1);        
            sm1.COPy = interpolate_array(limb1.COPy, ...
                        limb1.stopIndex(1)-tol1, limb1.startIndex(2)+tol1); 

        case 4
            % For limb1, interpolate between end of 1st hit & start of 2nd,
            %       and reflect about 2nd stopIndex to final time.
            sm1.COPx = interpolate_array(limb1.COPx, ...
                        limb1.stopIndex(1)-tol1, limb1.startIndex(2)+tol1); 
            sm1.COPy = interpolate_array(limb1.COPy, ...
                        limb1.stopIndex(1)-tol1, limb1.startIndex(2)+tol1); 
            sm1.COPx = ...
                  mirror_array(sm1.COPx, limb1.stopIndex(2)-tol1, tfIndex);                                     
            sm1.COPy = ...
                  mirror_array(sm1.COPy, limb1.stopIndex(2)-tol1, tfIndex);                                     
    end
    
    % Generate plot.
    figHandle = figHandleArray(1);
    if strcmpi(tInfo.limb(1), 'R')
        plot_GRFTzByLimb(GRFTz_byLimb.R, sm1, aTime, tol1, tInfo, figHandle);
        titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
            'GRFTz Data for R Limb:  Subject ', ...
            char(cSim.subject), '-', tInfo.trial, ', Speed ', ...
            tInfo.speed, ' m/s');
        Suptitle(titleString);    % MATLAB m-file for adding "super title"
    elseif strcmpi(tInfo.limb(1), 'L')
        plot_GRFTzByLimb(GRFTz_byLimb.L, sm1, aTime, tol1, tInfo, figHandle);
        titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
            'GRFTz Data for L Limb:  Subject ', ...
            char(cSim.subject), '-', tInfo.trial, ', Speed ', ...
            tInfo.speed, ' m/s');
        Suptitle(titleString);    % MATLAB m-file for adding "super title"
    end
    
    % Query user.
    query = 'Adjust Gap Tolerance?';
    opt1 = 'tolerance ++';
    opt2 = 'tolerance --';
    opt3 = 'print';
    opt4 = 'done';
    userInput = menu(query, opt1, opt2, opt3, opt4);
    switch userInput
        case 1
            tol1 = tol1 + 5;
        case 2
            tol1 = tol1 - 5;
        case 3
            orient(figHandle, 'tall');  
            printCommand = ['print -f', num2str(figHandle), ' -r600'];
            eval(printCommand);
        case 4
            done = 1;
    end
end
    

% Smooth and plot data for limb2 until user is satisfied.
done = 0;
while ~done
    
    % Smooth COP data, depending on number of FP hits.
    switch nHits     
        case 2  
            % For limb2, reflect about startIndex to initial time.
            sm2.COPx = ...
               mirror_array(limb2.COPx, limb2.startIndex(1)+tol2, tiIndex);
            sm2.COPy = ...
               mirror_array(limb2.COPy, limb2.startIndex(1)+tol2, tiIndex);

        case 3  
            % For limb2, reflect about startIndex to initial time, 
            %                   and about stopIndex to final time.
            sm2.COPx = ...
               mirror_array(limb2.COPx, limb2.startIndex(1)+tol2, tiIndex);
            sm2.COPy = ...
               mirror_array(limb2.COPy, limb2.startIndex(1)+tol2, tiIndex);        
            sm2.COPx = ...
               mirror_array(sm2.COPx, limb2.stopIndex(1)-tol2, tfIndex);
            sm2.COPy = ...
               mirror_array(sm2.COPy, limb2.stopIndex(1)-tol2, tfIndex);

        case 4
            % For limb2, interpolate between end of 1st hit & start of 2nd,
            %       and reflect about 1st startIndex to initial time.
            sm2.COPx = interpolate_array(limb2.COPx, ...
                        limb2.stopIndex(1)-tol2, limb2.startIndex(2)+tol2); 
            sm2.COPy = interpolate_array(limb2.COPy, ...
                        limb2.stopIndex(1)-tol2, limb2.startIndex(2)+tol2); 
            sm2.COPx = ...
                mirror_array(sm2.COPx, limb2.startIndex(1)+tol2, tiIndex);                                     
            sm2.COPy = ...
                mirror_array(sm2.COPy, limb2.startIndex(1)+tol2, tiIndex);                                     
    end
    
    % Generate plot.
    figHandle = figHandleArray(2);
    if strcmpi(tInfo.limb(2), 'R')
        plot_GRFTzByLimb(GRFTz_byLimb.R, sm2, aTime, tol2, tInfo, figHandle);
        titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
            'GRFTz Data for R Limb:  Subject ', ...
            char(cSim.subject), '-', tInfo.trial, ', Speed ', ...
            tInfo.speed, ' m/s');
        Suptitle(titleString);    % MATLAB m-file for adding "super title"
    elseif strcmpi(tInfo.limb(2), 'L')
        plot_GRFTzByLimb(GRFTz_byLimb.L, sm2, aTime, tol2, tInfo, figHandle);
        titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
            'GRFTz Data for L Limb:  Subject ', ...
            char(cSim.subject), '-', tInfo.trial, ', Speed ', ...
            tInfo.speed, ' m/s');
        Suptitle(titleString);    % MATLAB m-file for adding "super title"
    end
    
    % Query user.
    query = 'Adjust Gap Tolerance?';
    opt1 = 'tolerance ++';
    opt2 = 'tolerance --';
    opt3 = 'print';
    opt4 = 'done';
    userInput = menu(query, opt1, opt2, opt3, opt4);
    switch userInput
        case 1
            tol2 = tol2 + 5;
        case 2
            tol2 = tol2 - 5;
        case 3
            orient(figHandle, 'tall');  
            printCommand = ['print -f', num2str(figHandle), ' -r600'];
            eval(printCommand);
            done = 1;
        case 4
            done = 1;
    end  
end

% Store smoothed COP data in structure and return.
if strcmpi(tInfo.limb(1), 'R')
    COP_smoothed.R = sm1;
    COP_smoothed.L = sm2;
elseif strcmpi(tInfo.limb(1), 'L')
    COP_smoothed.L = sm1;
    COP_smoothed.R = sm2;
end    
return;