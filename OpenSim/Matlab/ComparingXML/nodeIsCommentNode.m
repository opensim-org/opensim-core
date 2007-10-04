function isCommentNode = nodeIsCommentNode( node )
%
% Returns true iff the node is a text node whose name is #comment.
%
% Chand T. John, September 2007
% Stanford University
%

isTextNode = nodeIsTextNode( node );
hasCommentName = strcmpi( node.Name, '#comment' );
isCommentNode = isTextNode && hasCommentName;
