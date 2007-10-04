function isAttributeNode = nodeIsAttributeNode( node )
% Returns true if the node is an attribute node, i.e. it is a leaf node
% that is not a text node.
isLeafNode = nodeIsLeafNode( node );
isTextNode = nodeIsTextNode( node );
isAttributeNode = isLeafNode && ~isTextNode;
