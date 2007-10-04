function theStruct = addNodeAddresses( ...
    theStruct,       ...
    parentAddress,   ...
    currentNodeIndex )
%
% Adds an "Address" field to each node representing its position in the
% tree.  This can be useful, for instance, for determining a node's parent
% without having to search through the whole tree.
%
% PARAMETERS
% theStruct is the node to be modified.
% parentAddress is the address field of the current node's parent node.
% currentNodeIndex is the index of the current node in the Children array
% of its parent node.
%
% Chand T. John, September 2007
% Stanford University
%

% This should only happen for the root node, whose parent does not exist,
% so the code that called this method would have had no parent address to
% pass for the root anyway.
if nargin < 2
    parentAddress = [];
    currentNodeIndex = [];
end

% Set current node's address to be the parent node's address, combined with
% the current node's index.
theStruct.Address = [ parentAddress currentNodeIndex ];

% Add addresses of current node's children.  If the current node is a leaf
% node, this code block essentially does nothing.
numChildren = nodeNumberOfChildren( theStruct );
for i = 1 : numChildren
    % Create an "Address" field for each child node so the next line
    % doesn't throw an error.
    theStruct.Children(i).Address = [];
    theStruct.Children(i) = addNodeAddresses( ...
        theStruct.Children(i), ...
        theStruct.Address,     ...
        i                      );
end
