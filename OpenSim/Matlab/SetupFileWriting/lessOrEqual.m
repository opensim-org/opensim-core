function b = lessOrEqual( floatingPointNumber1, floatingPointNumber2 )

% If floatingPointNumber1 and floatingPointNumber2 differ by less than this
% amount, then we will assume they are equal.
EPSILON = 0.000001;
absoluteDifference = abs( floatingPointNumber1 - floatingPointNumber2 );
numbers1And2AreEqual = absoluteDifference < EPSILON;

% n1 <= n2 iff n1 < n2 or n1 == n2
b = floatingPointNumber1 < floatingPointNumber2 || numbers1And2AreEqual;
