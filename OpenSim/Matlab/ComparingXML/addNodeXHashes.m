function tree = addNodeXHashes( tree )
% Adds an "XHash" field to each node in the input XML tree representing the
% whole subtree rooted at that node, for use in Wang et al.'s X-Diff
% algorithm for comparing two XML files.  Note that this XHash algorithm
% differs from DOMHash by Maruyama et al. (1998) "Digest values for DOM
% (DOMHash) proposal", IBM Tokyo Research Laboratory,
% http://www.trl.ibm.co.jp/projects/xml/domhash.htm,
% in that XHash ignores the ordering of the children of any node.

if nodeIsLeafNode( tree ) % tree is a text or attribute node
    tree.XHash = md5( [ tree.Type tree.Name tree.Value ] );
elseif nodeIsElementNode( tree )
    numChildren = nodeNumberOfChildren( tree );
    childXhashes = cell( 1, numChildren );
    for i = 1 : numChildren
        tree.Children(i).XHash = '';
        tree.Children(i) = addNodeXHashes( tree.Children(i) );
        childXhashes{i} = tree.Children(i).XHash;
    end
    childXhashes = sort( childXhashes );
    hashInputString = [ tree.Type tree.Name ];
    for i = 1 : numChildren
        hashInputString = [ hashInputString childXhashes{i} ];
    end
    tree.XHash = md5( hashInputString );
else
    error( 'Invalid node type!' );
end
