function leafNodes = nodeGetAllLeafNodes( tree )
%
% Given a node, this function returns an array containing all of the leaf
% nodes of this node.
%
% Chand T. John, September 2007
% Stanford University
%

if nodeIsLeafNode( tree )
    leafNodes = tree;
elseif nodeHasChildren( tree )
    leafNodes = [];
    numChildren = nodeNumberOfChildren( tree );
    for i = 1 : numChildren
        leafNodes = [ leafNodes nodeGetAllLeafNodes( tree.Children(i) ) ];
    end
else
    error( 'Invalid node!' );
end
