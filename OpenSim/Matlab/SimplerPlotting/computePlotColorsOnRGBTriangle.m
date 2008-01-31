function colorsToPlot = computePlotColorsOnRGBTriangle(N)
%
% Usage: colorsToPlot = computePlotColorsOnRGBTriangle(N)
%
% Returns a cell array containing N evenly spaced colors on the triangle
% connecting the points (1,0,0) (red), (0,0,1) (blue), and (0,1,0) (green)
% in the three-dimensional Euclidean RGB space.  If N == 1, then red is
% returned, and if N < 1, an error is thrown.
%

%
% Throw an error if N < 1 or if no argument was sent into this function.
%
if nargin < 1
    error( 'Need to specify a number N >= 1 of colors to return!' );
end
if N < 1
    error( 'Need at least one color for this function to work!' );
end

%
% If N == 1, by default, return the red color.
%
if N == 1
    colorsToPlot = { [ 1 0 0 ] };
    return;
end

%
% If we get this far, that means N is an integer greater than 1.
%

% Compute the fraction of the way between 0 and 1 that the N evenly spaced
% points should be on the RGB triangle curve, if this curve were in fact a
% curve parameterized from 0 to 1.  Note that since this triangle curve is
% a loop, the color with parameter 0 and with parameter 1 is the same
% point: the red color.  So the evenly spaced points we choose for a given
% N, for N = 2, 3, 4, and 5, for example, have the following parameters:
%
% N == 2 => Return colors with parameters 0, 1/2
% N == 3 => Return colors with parameters 0, 1/3, 2/3
% N == 4 => Return colors with parameters 0, 1/4, 2/4, 3/4
% N == 5 => Return colors with parameters 0, 1/5, 2/5, 3/5, 4/5
% and so on.
parameters = ( 0 : ( N - 1 ) ) / N;

% Given the parameters of the colors we want to return, compute the actual
% colors and return them!
colorsToPlot = computeRGBTriangleColors( parameters );
