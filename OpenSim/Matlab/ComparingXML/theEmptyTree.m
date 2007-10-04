function tree = theEmptyTree()
%
% Returns the empty tree.
%
% Chand T. John, September 2007
% Stanford University
%

tree = struct( 'Name', 'EmptyTree', 'Value', '' );
tree.Children = [];
