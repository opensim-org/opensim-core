function e = computeEditScript( tree1, tree2, minmatch, dt )
%
% This function computes an edit script for transforming tree1 to tree2,
% which is a subroutine of the X-Diff XML file differencing algorithm
% developed by Wang et al. (2003).  This function is called by
% compareSetupFiles.m.
%
% Chand T. John
% Stanford University
%

%
% Initialize edit script and tree root address variables.
%

e = '';
x = tree1.Address;
y = tree2.Address;

%
% Cover the simple cases.
%

% If the root nodes of tree1 and tree2 are not in minmatch, then the script
% for changing tree1 into tree2 is simply to delete all of tree1 and insert
% all of tree2.
numMatches = length( minmatch );
foundMatch = false;
for i = 1 : numMatches
    if twoArraysAreEqual( x, minmatch{i}{1} ) && ...
            twoArraysAreEqual( y, minmatch{i}{2} )
        foundMatch = true;
        % We found a match.  No need to search anymore!
        break;
    end
end
if ~foundMatch
    tree1String = nodeToString( tree1 );
    tree2String = nodeToString( tree2 );
    e = sprintf( '\nDeleted %s\nInserted %s', tree1String, tree2String );
    return;
end

% If the tree1 and tree2 are exactly equal, i.e. if the edit distance from
% tree1 to tree 2 is zero, then the edit script is nothing.
if dt( tree1.Number, tree2.Number ) == 0
    e = '';
    return;
end

%
% Compute edit script for all children of tree1 and tree2.
%

% Compute edit script for matching node pairs.
matchIndices1 = [];
matchIndices2 = [];
for i = 1 : numMatches
    address1 = minmatch{i}{1};
    address2 = minmatch{i}{2};
    if length( address1 ) == 1
        parentAddress1 = [];
    else % address1 is [] or has 2 or more elements
        % If address1 is [], parentAddress = empty 1x0 matrix, which is not
        % equal to [], so the twoArraysAreEqual function below will return
        % false if we compare parentAddress to any actual node's address,
        % including the root node's address which is [].
        parentAddress1 = address1( 1 : end - 1 );
    end
    if length( address2 ) == 1
        parentAddress2 = [];
    else % address2 is [] or has 2 or more elements
        % If address2 is [], parentAddress = empty 1x0 matrix, which is not
        % equal to [], so the twoArraysAreEqual function below will return
        % false if we compare parentAddress to any actual node's address,
        % including the root node's address which is [].
        parentAddress2 = address2( 1 : end - 1 );
    end
    if twoArraysAreEqual( parentAddress1, x ) && ...
            twoArraysAreEqual( parentAddress2, y )
        matchIndices1 = [ matchIndices1 address1( end ) ];
        matchIndices2 = [ matchIndices2 address2( end ) ];
        % Since the node with address1 in the original tree actually just
        % has address1( end ) in tree1, and the node with address2 in the
        % other original tree actually just has address2( end ) in tree2,
        % call nodeGetNodeByAddress with just the last element of each
        % child node's address list.
        child1 = nodeGetNodeByAddress( address1( end ), tree1 );
        child2 = nodeGetNodeByAddress( address2( end ), tree2 );
        if nodeIsLeafNode( child1 ) && nodeIsLeafNode( child2 )
            if dt( child1.Number, child2.Number ) ~= 0
                % The value is the only difference between child1 and
                % child2.
                child1String = nodeToString( child1 );
                e = sprintf( ...
                    '%s\nUpdated value of %s from %s to %s', ...
                    e, child1String, child1.Value, child2.Value );
            end
        else
            e = [ e computeEditScript( child1, child2, ...
                minmatch, dt ) ];
        end
    end
end

% Compute edit script for nodes in tree1 that are not in tree2.
noMatchIndices1 = ...
    setdiff( 1 : nodeNumberOfChildren( tree1 ), matchIndices1 );
for i = 1 : length( noMatchIndices1 )
    childIndex = noMatchIndices1(i);
    childString = nodeToString( tree1.Children( childIndex ) );
    e = sprintf( '%s\nDeleted %s', e, childString );
end

% Compute edit script for nodes in tree2 that are not in tree1.
noMatchIndices2 = ...
    setdiff( 1 : nodeNumberOfChildren( tree2 ), matchIndices2 );
for i = 1 : length( noMatchIndices2 )
    childIndex = noMatchIndices2(i);
    childString = nodeToString( tree2.Children( childIndex ) );
    e = sprintf( '%s\nInserted %s', e, childString );
end
