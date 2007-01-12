function outArray = mirror_array(inArray, mirrorIndex, gapIndex)
% Purpose:  Performs a mirror reflection of inArray about the point
%           specified by mirrorIndex, generating new array values 
%           between gapIndex and mirrorIndex.
%
%           NOTE:  If the gap is larger than the known data, this function
%                  performs a mirror reflection of the known data, then
%                  extrapolates to fill the gap.
%
% Input:    inArray is an array
%           mirrorIndex is the index of inArray about which to mirror data
%           gapIndex is the index of inArray that defines the upper/lower
%               limit, or 'gap' to fill with new reflected values
%
% Output:   outArray is a copy of inArray, except that points between 
%               gapIndex and mirrorIndex are filled with new values 
%               corresponding to the mirror reflection of inArray 
%               about mirrorIndex (extrapolated, if needed).
%
% Example:  To make the trajectory of the COP continuous,
%               let inArray = COPx array (assume 2nd FP hit), 
%               let mirrorIndex = analog frame corresponding to TO
%               let gapIndex = analog frame corresponding to end of
%                   simulateable segment
%
% Called Functions:
%       extrapolate_array(inArray, gapStartIndex, gapStopIndex)
% 
% ASA, 10-05


% Initialize output array.
outArray = inArray;

% Set flag indicating whether gap falls before or after data,
% and get dimensions of gap and data available for mirroring.
if mirrorIndex > gapIndex
    gapFlag = 'before';                 % gap falls before data
    nGapPts = mirrorIndex - gapIndex;
    nDataPts = length(inArray) - mirrorIndex;    
elseif mirrorIndex < gapIndex
    gapFlag = 'after';                  % gap falls after data
    nGapPts = gapIndex - mirrorIndex;
    nDataPts = mirrorIndex - 1;
end

% Set flag indicating whether extrapolation is needed or not needed,
% (i.e., whether gap is larger or smaller than available data).
if nDataPts >= nGapPts
    extrapFlag = 0;         % extrapolation not needed
else
    extrapFlag = 1;         % extrapolation needed
end    
  
% Perform mirror reflection and extrapolation, as needed.
switch extrapFlag
    case 0                  % mirror reflection        
        % Set index counter.
        if strcmpi(gapFlag, 'before')
            indexCounter = gapIndex:(mirrorIndex - 1);
        elseif strcmpi(gapFlag, 'after')
            indexCounter = gapIndex: -1 :(mirrorIndex + 1);
        end
        
        % Perform mirror reflection between gapIndex and mirrorIndex.
        for i = indexCounter
            delta = mirrorIndex - i;
            outArray(i) = ...
                2*inArray(mirrorIndex) - inArray(mirrorIndex + delta);
        end
        
    case 1                  % mirror reflection and extrapolation
        % Set index counter.
        if strcmpi(gapFlag, 'before')
            availIndex = mirrorIndex - nDataPts;
            indexCounter = availIndex:(mirrorIndex - 1);
        elseif strcmpi(gapFlag, 'after')
            availIndex = mirrorIndex + nDataPts;
            indexCounter = availIndex: -1 :(mirrorIndex + 1);
        end
        
        % Perform mirror reflection between availIndex and mirrorIndex.
        for i = indexCounter
            delta = mirrorIndex - i;
            outArray(i) = ...
                2*inArray(mirrorIndex) - inArray(mirrorIndex + delta);
        end
        
        % Extrapolate between gapIndex and availIndex.
        if strcmpi(gapFlag, 'before')
            outArray = extrapolate_array(outArray, gapIndex, availIndex);
        elseif strcmpi(gapFlag, 'after')
            outArray = extrapolate_array(outArray, availIndex, gapIndex);
        end
end
return;        
    