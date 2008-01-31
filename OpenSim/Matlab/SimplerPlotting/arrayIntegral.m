function F = arrayIntegral( f, time )
%
% Usage: F = arrayIntegral( f, time )
%
% Here, time is a column vector, f is a column vector or array with the
% same number of rows as time, and F will also be a column vector or array
% with the same numbers of rows and columns as f.  f represents a function
% (a scalar function if f is a column vector, or a vector function if f is
% an array with more than one column) that is the linear interpolation of
% the points ( time(1), f(time(1)) ), ..., ( time(N), f(time(N)) ), where
% N == length( time ).  F represents the exact integral of the linear
% interpolation function f, though f itself may by an affine approximation
% of some more complicated function.
%
% f is integrated using the mean value theorem, which states that the
% integral of a function f(t) from t1 to t2 is f_avg * (t2-t1), where f_avg
% is the average value of the function f(t) for t ranging from t1 to t2.
% For each line segment between two adjacent points (t1, f1) and (t2, f2),
% where t1 < t2, in the graph of the linear interpolation function f, the
% average value of f from t1 to t2 is obviously just f_avg = (f1+f2)/2.
% Then the integral of f(t) from t1 to t2 is simply f_avg * (t2-t1) =
% (f1+f2)*(t2-t1)/2.  F(t) is defined as the integral of f(t) from its very
% first time value, time(1), to t, which can be computed as the sum of the
% integrals of f(t) for each interval in between times time(1) and t.
%
% In general, we can think of the array f as a vector function, where each
% row contains the value of f at some time, and similarly, the integral F
% will be an array of the same size, where each row contains the value of F
% at some particular time.  Note that the integral of f at time(1) itself
% is simply 0, since there is no area under the curve of f before time
% time(1).
%

% Initialize F.
F = zeros( size(f) );

% Compute F = the integral of f from time(1) to time( end ).
for j = 2 : length( time )
    
    %
    % Compute F( j, : ) = the integral of f from time(1) to time(j).
    %
    
    % Compute the average value of f from time( j - 1 ) to time(j).
    fAverage = mean( f( j - 1 : j, : ), 1 );
    
    % Compute the length of this time interval.
    lengthOfThisTimeInterval = time(j) - time( j - 1 );
    
    % Compute the integral of f in this time interval.
    integralInThisTimeInterval = fAverage * lengthOfThisTimeInterval;
    
    % Add the integral of f from time( j - 1 ) to time(j) to the integral
    % of f from time(1) to time( j - 1 ) to get the integral of f from
    % time(1) to time(j), which is what F( j, : ) is defined to be.
    F( j, : ) = F( j - 1, : ) + integralInThisTimeInterval;
    
end
