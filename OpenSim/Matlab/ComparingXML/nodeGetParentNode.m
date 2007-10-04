function parent = nodeGetParentNode( node, tree )
%
% Returns the parent node of the current node in its tree.
%
% NOTE: the parent node is found using the address field of the node, so
% you cannot in general use this function to find a particular node in any
% arbitrary tree, and you also cannot use this function if the tree has
% been modified since the node addresses were added to the nodes of the
% tree.
%
% Chand T. John, September 2007
% Stanford University
%

parentAddress = node.Address( 1 : end - 1 );
parent = nodeGetNodeByAddress( parentAddress, tree );
