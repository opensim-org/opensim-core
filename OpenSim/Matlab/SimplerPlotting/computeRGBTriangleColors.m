function colorsToPlot = computeRGBTriangleColors( parameters )
%
% Usage: colorsToPlot = computeRGBTriangleColors( parameters )
%
% Returns colors on the RGB triangle with the given parameter array.  The
% RGB triangle is parameterized from 0 to 1, with the point with parameter
% 0 and 1 being red, and points with all parameters between 0 and 1 are
% those along the RGB triangle on the path from red to blue to green and
% back to red.  If parameters outside of the interval [0, 1) are given, we
% simply wrap around and return the appropriate color.  So for example, the
% parameters -2, -1, 0, 1, 2, and 3 all correspond to the red color, and
% the parameters 1/3, -2/3, and -5/3 all correspond to the blue color.
%

%
% If any parameters are outside of the [0 1) range are given, then replace
% those parameters with equivalent parameters that are within the [0, 1)
% range.  Essentially, we just add the appropriate integer to the parameter
% value until it gets to be inside the [0, 1) range.  For example:
%
% parameter is in [-2, -1) => parameter = parameter + 2
% parameter is in [-1, 0 ) => parameter = parameter + 1
% parameter is in [ 0, 1 ) => parameter = parameter + 0
% parameter is in [ 1, 2 ) => parameter = parameter - 1;
%
% This is accomplished by:
% (1) determining which of these ranges each parameter is in, and then
% (2) adding by the appropriate integer to get the parameter into the
%     [0, 1) range.
%
% To do (1), we simply compute floor( parameter ), which gives us the
% smallest number in the range the parameter lies in, so for example if the
% parameter is -1/3, floor( parameter) gives us -1, so we know the
% parameter is in the [-1,0) range.
%
% To do (2), we replace parameter with parameter - floor( parameter ), so
% for example if parameter == -1/3, floor( parameter ) == -1, so
% parameter - floor( parameter ) = -1/3 - (-1) = 2/3, which is now in the
% [0,1) range.
%
% Fortunately we can do (1) and (2) for each parameter using a single
% vectorized MATLAB statement.
%
parameters = parameters - floor( parameters );

%
% Compute the color on the RGB triangle corresponding to each parameter in
% the parameters array and assign each color to the corresponding entry in
% the colorsToPlot cell array.
%
colorsToPlot = cell( length( parameters ), 1 );
for i = 1 : length( parameters )
    parameter = parameters(i);
    if 0 <= parameter && parameter < 1/3
        % If parameter is between 0 and 1/3, assign color to be the color
        % the appropriate fraction of the way from red to blue.
        t = parameter * 3;
        color = [ ( 1 - t ) 0 t ];
    elseif parameter < 2/3
        % If parameter is between 1/3 and 2/3, assign color to be the color
        % the appropriate fraction of the way from blue to green.
        t = ( parameter - 1/3 ) * 3;
        color = [ 0 t ( 1 - t ) ];
    elseif parameter < 1
        % If parameter is between 2/3 and 1, assign color to be the color
        % the appropriate fraction of the way from green to red.
        t = ( parameter - 2/3 ) * 3;
        color = [ t ( 1 - t ) 0 ];
    end
    colorsToPlot{i} = color;
end
