function theStruct = addSubtreeSizes( theStruct )
%
% Adds a "SubtreeSize" field to each node in theStruct, representing the
% number of nodes in the subtree rooted at each node.
%
% Chand T. John, September 2007
% Stanford University
%

if nodeIsLeafNode( theStruct )
    theStruct.SubtreeSize = 1;
else
    theStruct.SubtreeSize = 1;
    numChildren = nodeNumberOfChildren( theStruct );
    for i = 1 : numChildren
        theStruct.Children(i).SubtreeSize = -1;
        theStruct.Children(i) = addSubtreeSizes( theStruct.Children(i) );
        theStruct.SubtreeSize = ...
            theStruct.SubtreeSize + ...
            theStruct.Children(i).SubtreeSize;
    end
end
