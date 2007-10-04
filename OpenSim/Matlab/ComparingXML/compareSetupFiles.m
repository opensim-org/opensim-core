function outputString = compareSetupFiles( file1, file2, ...
    ignoreComments, ignoreUnimportantWhitespaces, ignoreExtraZeros )
%
% Utility to compare two setup files that are supposed to have the same
% basic structure, but one file may have properties that the other file
% doesn't, and the values of some properties that the files do share may
% differ.  This comparison ignores the order of the tags in any node of the
% XML file.
%
% This is an implementation of the X-Diff algorithm for unordered
% comparison of XML files, from the following paper:
% 
% Yuan Wang, David J. DeWitt, Jin-yi Cai
% X-Diff: A Fast Change Detection Algorithm for XML Documents, in Umeshwar
% Dayal, Krithi Ramamritham, T. M. Vijayaraman (Eds.): Proceedings of the
% 19th International Conference on Data Engineering (ICDE), Bangalore,
% India, March 5-8, 2003.
%
% USAGE:
% 1. compareSetupFiles( file1, file2 )
% 2. compareSetupFiles( file1, file2, includeComments )
% 3. outputString = compareSetupFiles( file1, file2 )
% 4. outputString = compareSetupFiles( file1, file2, includeComments )
%
% INPUT:
% 1. file1 and file2 are strings (character arrays) representing the full
%    or relative file names of the two setup files to compare.
% 2. ignoreComments is a Boolean variable indicating whether or not the
%    comparison should include comparing comments in the XML files or not.
% 3. ignoreUnimportantWhitespaces is a Boolean variable indicating whether
%    or not the comparison should consider whitespaces in the files that
%    are typically not important: namely, extra whitespaces at the
%    beginning or end of a text field, or any text field whose only value
%    is a set of whitespaces.
% 4. ignoreExtraZeros is a Boolean variable indicating whether or not the
%    comparison should report differences like "2" being changed to "2.0",
%    i.e. changes to numbers that the == operator would consider
%    unimportant.
%
% OUTPUT:
% A string listing the names and values of the properties that:
% 1. are in file1 but not in file2,
% 2. are in file2 but not in file1, and
% 3. are in both file1 and file2, but have different values.
%
% Chand T. John, September 2007
% Stanford University
%

if nargin < 3
    ignoreComments = true;
    ignoreUnimportantWhitespaces = true;
    ignoreExtraZeros = true;
end

%
% Read the XML files into Document Object Model (DOM) trees.
%

tree1 = parseXML( file1 );
tree2 = parseXML( file2 );

%
% Preprocess the trees for comments, whitespaces, and extra zeros.
%

% Remove comments if desired.
tree1 = removeComments( tree1 );
tree2 = removeComments( tree2 );

% Trim leading and trailing whitespaces in all Name and Value fields.
tree1 = removeEndWhitespaces( tree1 );
tree2 = removeEndWhitespaces( tree2 );

% Convert all numbers to uniform double-precision representation.
tree1 = uniformlyRepresentNumbers( tree1 );
tree2 = uniformlyRepresentNumbers( tree2 );

%
% Sort the children of each node in each tree, so that all future
% processing has no dependence on the actual order of tags as they appear
% in each file.  Sorting is done based on Type, then Name, and then Value.
%

tree1 = sortChildNodes( tree1 );
tree2 = sortChildNodes( tree2 );

%
% Compute hash strings for each of the two DOM trees.
%

tree1 = addNodeXHashes( tree1 );
tree2 = addNodeXHashes( tree2 );

%
% Check whether root nodes are equivalent, and remove any second-level
% nodes that are equivalent to cut down the number of nodes that are
% compared during the rest of this algorithm.
%

% If the two trees are equivalent, then say so and exit!
if strcmp( tree1.XHash, tree2.XHash )
    outputString = 'The two files are equivalent!';
    return;
end

% From this point on, we know tree1 and tree2 are not equivalent.  Now we
% check second-level nodes for equivalence and remove the pairs that are
% equivalent.
i = 1;
j = 1;
while i <= nodeNumberOfChildren( tree1 )
    while j <= nodeNumberOfChildren( tree2 )
        if strcmp( tree1.Children(i).XHash, tree2.Children(j).XHash )
            tree1.Children(i) = [];
            tree2.Children(j) = [];
            break;
        else
            j = j + 1;
        end
    end
    if j > nodeNumberOfChildren( tree2 )
        i = i + 1;
    end
end

%
% We now find a minimum-cost matching between tree1 and tree2.
%

% Compute the edit distance from tree1 to tree2.
% This code block uses a dynamic programming approach to compute the edit
% distance from tree1 to tree2, as shown in Wang et al. (2003).  Note that
% the implementation below should work for incomplete trees (i.e. trees
% with subtrees of varying depth), since eventually the proper parent nodes
% will be updated to their correct values, even if they are given incorrect
% values during an intermediate step in the code.
tree1 =  addNodeAddresses( tree1 );
tree2 =  addNodeAddresses( tree2 );
tree1 =    addNodeNumbers( tree1 );
tree2 =    addNodeNumbers( tree2 );
tree1 =   addSubtreeSizes( tree1 );
tree2 =   addSubtreeSizes( tree2 );
tree1 = addNodeSignatures( tree1 );
tree2 = addNodeSignatures( tree2 );
n1 =  nodeGetAllLeafNodes( tree1 );
n2 =  nodeGetAllLeafNodes( tree2 );
dt = -1 * ones( tree1.LargestNumberInSubtree, tree2.LargestNumberInSubtree );
while ~isempty( n1 ) && ~isempty( n2 )
    n2Indices = 1 : length( n2 );
    for i = 1 : length( n1 )
        p = 1;
        while p <= length( n2Indices )
            j = n2Indices(p);
            if strcmp( n1(i).Signature, n2(j).Signature )
                x = n1(i).Number;
                y = n2(j).Number;
                if nodeIsLeafNode( n1(i) ) && nodeIsLeafNode( n2(j) )
                    dt( x, y ) = ~strcmp( n1(i).Value, n2(j).Value );
                elseif nodeHasChildren( n1(i) ) && nodeHasChildren( n2(j) )
                    dt( x, y ) = 0;
                    numN1iChildren = nodeNumberOfChildren( n1(i) );
                    numN2jChildren = nodeNumberOfChildren( n2(j) );
                    n2jChildIndices = 1 : numN2jChildren;
                    u = 1;
                    for k = 1 : numN1iChildren
                        r = 1;
                        while r <= length( n2jChildIndices )
                            m = n2jChildIndices(r);
                            n1iChildNum = n1(i).Children(k).Number;
                            n2jChildNum = n2(j).Children(m).Number;
                            % If these two children match, then that match
                            % was previously recorded in dt.  So, add their
                            % distance to the distance between n1(i) and
                            % n2(j) if this is the case.
                            dist = dt( n1iChildNum, n2jChildNum );
                            if dist >= 0 % found a match!
                                n1iMatchedChildren(u) = k; %#ok<AGROW>
                                n2jMatchedChildren(u) = m; %#ok<AGROW>
                                u = u + 1;
                                dt( x, y ) = dt( x, y ) + dist;
                                % If the mth child of n2(j) matched the kth
                                % child of n1(i), then don't ever compare
                                % the mth child of n2(j) to any other child
                                % of n1(i), since only at most one child of
                                % n1(i) is allowed to be compared to the
                                % mth child of n2(j) anyway.
                                n2jChildIndices(r) = [];
                                % For the same reason, don't compare the
                                % kth child of n1(i) to be compared to any
                                % other child of n2(j).
                                break;
                            else
                                r = r + 1;
                            end
                        end
                    end
                    n1iUnmatchedChildren = setdiff(        ...
                        1 : nodeNumberOfChildren( n1(i) ), ...
                        n1iMatchedChildren                 );
                    n2jUnmatchedChildren = setdiff(        ...
                        1 : nodeNumberOfChildren( n2(j) ), ...
                        n2jMatchedChildren                 );
                    numN1iUnmatchedChildren = length( n1iUnmatchedChildren );
                    numN2jUnmatchedChildren = length( n2jUnmatchedChildren );
                    for k = 1 : numN1iUnmatchedChildren
                        childIndex = n1iUnmatchedChildren(k);
                        child = n1(i).Children( childIndex );
                        dt( x, y ) = dt( x, y ) + child.SubtreeSize;
                    end
                    for k = 1 : numN2jUnmatchedChildren
                        childIndex = n2jUnmatchedChildren(k);
                        child = n2(j).Children( childIndex );
                        dt( x, y ) = dt( x, y ) + child.SubtreeSize;
                    end
                else
                    error( [ 'Leaf vs. non-leaf comparison!' ...
                        'This is not supposed to happen.' ] );
                end
                % Since we found a match between n1(i) and n2(j), and we
                % assume each node in n1 matches with at most one node in
                % n2, we don't want to compare n2(j) to any other node in
                % n1 ever again.
                n2Indices(p) = [];
                % For the same reason, we don't want to compare n1(i) to
                % any other node in n2 ever again.
                break;
            else
                p = p + 1;
            end
        end
    end
    n1 = nodeGetAllParentNodes( n1, tree1 );
    n2 = nodeGetAllParentNodes( n2, tree2 );
end

% Compute matchings from tree1 to tree2.
% This code block uses the minimum-cost maximum flow algorithm to compute
% the minimum-cost bipartite matching between subtrees.
minmatch = {};
if strcmp( tree1.Signature, tree2.Signature )
    minmatch{1} = { tree1.Address, tree2.Address };
    i = 1;
    while i <= length( minmatch )
        matchAddress1 = minmatch{i}{1};
        matchAddress2 = minmatch{i}{2};
        node1 = nodeGetNodeByAddress( matchAddress1, tree1 );
        node2 = nodeGetNodeByAddress( matchAddress2, tree2 );
        numChildren1 = nodeNumberOfChildren( node1 );
        numChildren2 = nodeNumberOfChildren( node2 );
        node2Indices = 1 : numChildren2;
        for k = 1 : numChildren1
            p = 1;
            while p <= length( node2Indices )
                m = node2Indices(p);
                child1 = node1.Children(k);
                child2 = node2.Children(m);
                x = child1.Number;
                y = child2.Number;
                if dt( x, y ) >= 0 % the two children match!
                    a1 = child1.Address;
                    a2 = child2.Address;
                    minmatch{ length( minmatch ) + 1 } = { a1 a2 };
                    node2Indices(p) = [];
                    break;
                else
                    p = p + 1;
                end
            end
        end
        i = i + 1;
    end
end

%
% We generate a minimum-cost edit script from tree1 to tree2 based on the
% matching we just computed.
%

editScript = computeEditScript( tree1, tree2, minmatch, dt );

%
% Form string representing the result of comparing the two setup files.
%

newline = sprintf( '\n' );

outputString = [];
outputString = [ outputString 'file1 = ' file1 newline ];
outputString = [ outputString 'file2 = ' file2 newline ];
outputString = [ outputString newline ];
outputString = [ outputString editScript ];
