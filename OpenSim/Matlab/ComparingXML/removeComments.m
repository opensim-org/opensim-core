function tree = removeComments( tree )
%
% Removes all #comment nodes from parsed XML tree.  Note, if the root node
% of the tree is a comment, we do not delete it.  So, this function should
% never be called when tree is a comment node.
%
% Chand T. John, September 2007
% Stanford University
%

if nodeIsLeafNode( tree )
    % Tree is an attribute node or a text node, such as a comment.
    % Make no changes to tree.
elseif nodeIsElementNode( tree )
    % Tree has children.  Remove any children that are comments, or, for
    % any children that are not leaf nodes, remove any of their descendants
    % that are comments.
    i = 1;
    while i <= nodeNumberOfChildren( tree )
        child = tree.Children(i);
        if nodeIsCommentNode( child )
            % If child is a comment node, delete it!
            tree.Children(i) = [];
            % Next child becomes tree.Children(i), so don't increment i.
        else
            if nodeIsLeafNode( child )
                % Child is an attribute node or non-comment text node.
                % Do nothing.
            elseif nodeIsElementNode( child )
                % Remove any descendants of child that are comments.
                tree.Children(i) = removeComments( child );
            else
                % This should never happen.
                error( 'Invalid node type!' );
            end
            i = i + 1;
        end
    end
else
    % This should never happen.
    error( 'Invalid node type!' );
end
