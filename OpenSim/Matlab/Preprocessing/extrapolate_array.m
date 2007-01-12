function outArray = extrapolate_array(inArray, gapStartIndex, gapStopIndex)
% Purpose:  Performs one-dimensional extrapolation at the beginning or end
%           of inArray between gapStartIndex and gapStopIndex.
%    
% Input:    inArray is an array
%           gapStartIndex is the index of 1st point of the gap
%           gapStopIndex is the index of the final point of the gap
%
% Notes:    1. the function assumes that gapStartIndex < gapStopIndex
%           2. the function assumes that either gapStartIndex = 1, or
%              gapStopIndex = length(inArray).
%
% Output:   outArray is a copy of inArray, except that points between 
%               gapStartIndex and gapStopIndex are extrapolated.
% 
% ASA, 10-05


% Specify interpolation method to use with Matlab's interp1 command.
method = 'cubic';

% Initialize output array.
outArray = inArray;

% Define data arrays; 
% these depend on whether the gap falls before or after data.
if gapStartIndex == 1                       % gap falls before data
    x = (gapStopIndex+1):length(inArray);
    y = inArray(x);
elseif gapStopIndex == length(inArray);     % gap falls after data
  x = 1:(gapStartIndex-1);  
  y = inArray(x);
end

% Perform interpolation.
gapIndices = gapStartIndex:gapStopIndex;
gapValues = interp1(x, y, gapIndices, method);

% Store data in output structure and return.
outArray(gapIndices) = gapValues;
return;