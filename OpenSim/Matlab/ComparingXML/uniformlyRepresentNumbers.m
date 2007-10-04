function tree = uniformlyRepresentNumbers( tree )
%
% Convert all numbers to uniform double-precision representation in the
% Value field of all nodes.  Element nodes are checked as well as text and
% attribute nodes.  Note, currently any whitespaces between strings within
% the Value field of each node will be converted to a single whitespace.
%
% Chand T. John, September 2007
% Stanford University
%

%
% Uniformize representation of all numbers in Value field of current node.
% Note, there could be multiple numbers mixed with strings in some value
% fields, so we need to uniquely identify all numerical substrings
% separated by whitespaces and replace those numbers with strings that have
% a uniform double-precision representation that MATLAB would choose.  Our
% strategy is to break up the Value field of the current node into a cell
% array of strings, where we assume individual strings are separated by
% whitespaces, and then we run str2double and num2str on the cell array to
% convert any numbers into a uniform format.
%

% Break up tree.Value into a cell array, assuming each string is separated
% from the others by whitespaces only.
str = tree.Value;
words = {};
while ~isempty( str )
    [ token, remain ] = strtok( str );
    words{ length( words ) + 1 } = token;
    str = remain;
end

% For each string in the cell array that is actually a number, convert it
% to a standard MATLAB double-precision representation.
for i = 1 : length( words )
    n = str2double( words{i} );
    if ~isnan(n) % current word is a number
        % Replace number in cell array with uniform representation of the
        % number.
        words{i} = num2str(n);
    end
end

% Set tree.Value to be the strings in the modified cell array, separated by
% single space characters.
tree.Value = '';
for i = 1 : length( words )
    if i == length( words )
        tree.Value = [ tree.Value words{i} ];
    else
        tree.Value = [ tree.Value words{i} ' ' ];
    end
end

%
% Do the same for any descendants of this node.
%

% If current node has no children, this code essentially does nothing.
numChildren = nodeNumberOfChildren( tree );
for i = 1 : numChildren
    tree.Children(i) = uniformlyRepresentNumbers( tree.Children(i) );
end
