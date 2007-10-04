function node = nodeGetNodeByAddress( address, tree )
%
% Given the address of a node in a tree, this function returns that node.
%
% Chand T. John, September 2007
% Stanford University
%

n = length( address );
node = tree;
for i = 1 : n
    node = node.Children( address(i) );
end
