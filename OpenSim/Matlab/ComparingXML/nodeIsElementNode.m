function isElementNode = nodeIsElementNode( node )
% Returns true iff a node is not a leaf node, i.e. iff the node has
% children.
isElementNode = nodeHasChildren( node );
