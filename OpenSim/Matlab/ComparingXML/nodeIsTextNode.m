function isTextNode = nodeIsTextNode( node )
% Returns true if the node is a text node, i.e. it is a leaf node whose
% name is either #text or #comment.
isLeafNode = nodeIsLeafNode( node );
nameIsTextOrComment =                   ...
    strcmpi( node.Name, '#text'    ) || ...
    strcmpi( node.Name, '#comment' );
% TO DO - are there other types of text nodes, like CDATA??  If so, add
% those to the line above.
isTextNode = isLeafNode && nameIsTextOrComment;
