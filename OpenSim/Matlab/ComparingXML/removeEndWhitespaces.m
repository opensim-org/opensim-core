function tree = removeEndWhitespaces( tree )
%
% Removes leading and trailing whitespaces in the Name and Value fields of
% every node in the tree.
%
% Chand T. John, September 2007
% Stanford University
%

% Remove whitespaces from current node's Name and Value fields.
tree.Name  = strtrim( tree.Name  );
tree.Value = strtrim( tree.Value );

% Remove whitespaces from Name and Value field of all descendants.
% If this node has no children, the code below essentially does nothing.
numChildren = nodeNumberOfChildren( tree );
for i = 1 : numChildren
    tree.Children(i) = removeEndWhitespaces( tree.Children(i) );
end
