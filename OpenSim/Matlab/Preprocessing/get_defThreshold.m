function defThreshold = get_defThreshold(dX, dt, limb, eFromGRF, c, tInfo)
% Purpose:  Computes a 'default' threshold for detecting events from
%           velocity data dX, based on the timing of 'known' events 
%           from vertical GRF data.
%
% Input:    dX is an array of velocity values
%           dt is an array of times corresponding to dX
%           limb is the limb of interest corresponding to dX
%           eFromGRF(fpHitNum) is an array of IC or TO events, 
%               in analog frames, one per FP hit, in the order 
%               specified by tInfo.FP
%           c is a structure returned from read_c3DFile()
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.limb  - limbs corresponding to FP strikes (cell array)
%
% Output:   defThreshold is the average dX value at known events from
%               eFromGRF, for the limb of interest.
%
% ASA, 9-05


% Get elements of eFromGRF corresponding to limb of interest,
%   and convert from analog frames to time.
limbIndices = strmatch(limb, tInfo.limb);
eTimes = eFromGRF(limbIndices)/c.analog.rate;

% Get value of dX at known events.
for eNum = 1:length(eTimes)
    dtIndex = min(find(dt >= eTimes(eNum)));
    dX_at_eTimes(eNum) = dX(dtIndex);
end

% Define default threshold for detecting other IC events from dX.
defThreshold = mean(dX_at_eTimes);
return;
