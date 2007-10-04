function parents = nodeGetAllParentNodes( nodes, tree )
%
% This function returns an array of nodes: the parent nodes of all nodes
% passed into this function, in the given tree, with no repeats.  The
% parameter "nodes" must be an array of nodes from the tree passed into
% this function.
%
% NOTE: each parent node is found using the address field of the node, so
% you cannot in general use this function to find a particular node in any
% arbitrary tree, and you also cannot use this function if the tree has
% been modified since the node addresses were added to the nodes of the
% tree.
%
% Chand T. John, September 2007
% Stanford University
%

% Compute address of each node's parent.  If a node in the nodes array is
% the root node, just ignore it: don't compute a parent address for it.
numNodes = length( nodes );
parentAddresses = {};
for i = 1 : numNodes
    nodeAddress = nodes(i).Address;
    if length( nodeAddress ) > 1
        parentAddresses{ length( parentAddresses ) + 1 } = ...
            nodeAddress( 1 : end - 1 );
    elseif length( nodeAddress ) == 1
        parentAddresses{ length( parentAddresses ) + 1 } = [];
    end
end

% Remove all repeated addresses.
i = 1;
while i <= length( parentAddresses )
    address = parentAddresses{i};
    j = i + 1;
    while j <= length( parentAddresses )
        if twoArraysAreEqual( address, parentAddresses{j} )
            parentAddresses(j) = [];
        else
            j = j + 1;
        end
    end
    i = i + 1;
end
% Now parentAddresses contains a set of unique addresses, each address
% being the address of a parent node of one or more of the nodes passed
% into this function.

% We now grab and pack all of these parent nodes into a single array to be
% returned from this function.
if length( parentAddresses ) < 1
    parents = [];
else
    for i = 1 : length( parentAddresses )
        parents(i) = nodeGetNodeByAddress( parentAddresses{i}, tree );
    end
end
