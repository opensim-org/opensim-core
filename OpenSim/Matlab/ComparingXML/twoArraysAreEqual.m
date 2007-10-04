function areEqual = twoArraysAreEqual( a1, a2 )
%
% Returns true iff a1 and a2 are equal arrays consisting of elements that
% can be compared using the == operator, such as doubles or characters.
%
% Chand T. John, September 2007
% Stanford University
%

%
% If arrays don't have equal dimensions (sizes), return false.
%

% Check that the "array size" arrays have the same dimensions!
a1size = size( a1 );
a2size = size( a2 );
if length( a1size ) ~= length( a2size )
    areEqual = false;
    return;
end

% Check that the "array size" arrays have the same elements.
numDimensions = length( a1size );
for i = 1 : numDimensions
    if a1size(i) ~= a2size(i)
        areEqual = false;
        return;
    end
end

%
% If we got this far, that means the arrays a1 and a2 have the same
% dimensions.  Now we compare the individual elements of a1 and a2 using
% the == operator.  Thus, this function only works on arrays containing
% elements that can be compared using the == operator, such as doubles or
% characters.
%

% This is the number of elements in both arrays.
n = prod( a1size );

% Compare all elements of a1 and a2 to get a matrix of 1's and 0's
% indicating whether or not each element of a1 is equal to its
% corresponding element in a2.  Have a special case for the empty array.
if n == 0
    % Both arrays have the same dimensions, and both have no elements, so
    % they are equal.  We can't use == on these arrays since that will just
    % return an empty array instead of a 1 or 0.
    areEqual = true;
else
    areEqual = ( a1 == a2 );
end

% Collapse the matrix of 1's and 0's until a single Boolean value is left.
for i = 1 : n - 1
    areEqual = all( areEqual );
end
