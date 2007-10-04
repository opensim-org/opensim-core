function newStruct = addRootNodeIfNecessary( theStruct )
%
% Given an array of trees, theStruct, representing a parsed XML file, this
% function checks whether theStruct has a single root node, and if not,
% adds one, thus making each tree in theStruct a subtree of that root node.
%
% Chand T. John, September 2007
% Stanford University
%

% If there is no more than one tree in theStruct, do nothing.
numTrees = length( theStruct );
if numTrees < 2
    newStruct = theStruct;
    return;
end

% There is more than one tree in theStruct.  So, create a root node and
% make each tree a subtree of this new root node.
newStruct = struct( 'Name', 'RootNode', ...
    'Value', 'I was not part of the original XML file!' );
for i = 1 : numTrees
    newStruct.Children(i) = theStruct(i);
end
