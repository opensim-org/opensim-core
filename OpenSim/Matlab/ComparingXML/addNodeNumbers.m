function tree = addNodeNumbers( tree, previousNumber )
%
% Adds a unique number to each node in the tree, given the current node's
% parent node's previous child's subtree's largest number.  The ndoes are
% numbered in the order of a pre-order traversal of the tree.  This
% function actually adds a "Number" field and a "LargestNumberInSubtree"
% field to each node in the tree.
%
% Chand T. John, September 2007
% Stanford University
%

% This should only happen for the root node of the tree, which has no
% parent.
if nargin < 2
    previousNumber = 0;
end

% The current node's number is the previous node's number plus one.
tree.Number = previousNumber + 1;

% Assign numbers to the children of the current node.
numChildren = nodeNumberOfChildren( tree );
for i = 1 : numChildren
    % Create "Number" and "LargestNumberInSubtree fields for child so the
    % assignment after the recursive call at the end of this for loop
    % doesn't throw an error.
    tree.Children(i).Number = -1;
    tree.Children(i).LargestNumberInSubtree = -1;
    % Compute the previous number for the ith child of the current node.
    % This number is the largest number in the previous child's subtree,
    % unless i == 1, in which case, the first child's number is of course
    % one plus the current node's number.
    if i == 1
        prevNumForChild = tree.Number;
    else
        prevNumForChild = tree.Children( i - 1 ).LargestNumberInSubtree;
    end
    % Number the ith child and all of its children.
    tree.Children(i) = addNodeNumbers( tree.Children(i), prevNumForChild );
end

% Assign "LargestNumberInSubtree" field to current node.
if numChildren > 0
    tree.LargestNumberInSubtree = ...
        tree.Children( numChildren ).LargestNumberInSubtree;
else % Current node is a leaf node
    tree.LargestNumberInSubtree = tree.Number;
end
