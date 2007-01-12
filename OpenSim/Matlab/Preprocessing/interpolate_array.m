function outArray = interpolate_array(inArray, gapStartIndex, gapStopIndex)
% Purpose:  Performs one-dimensional interpolation of inArray between
%           gapStartIndex and gapStopIndex.
%
% Input:    inArray is an array
%           gapStartIndex is the index of 1st point of the gap
%           gapStopIndex is the index of the final point of the gap
%
% Output:   outArray is a copy of inArray, except that points between 
%               gapStartIndex and gapStopIndex are interpolated.
% 
% ASA, 10-05


% Specify interpolation method to use with Matlab's interp1 command.
method = 'cubic';

% Initialize output array.
outArray = inArray;

% Perform interpolation.
x = [1:(gapStartIndex-1), (gapStopIndex+1):length(inArray)];
y = inArray(x);
gapIndices = gapStartIndex:gapStopIndex;
gapValues = interp1(x, y, gapIndices, method);

% Store data in output structure and return.
outArray(gapIndices) = gapValues;
return;