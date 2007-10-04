function theStruct = addNodeSignatures( theStruct, parentSignature )
%
% Adds "Signature" field to each node in the tree.  This function assumes
% that each node in the tree has a "Type" field.
%
% PARAMETERS
% theStruct is the node to be modified.
% parentSignature is the Signature field of the current node's parent node.
%
% Chand T. John, September 2007
% Stanford University
%

% This should only happen for the root node, whose parent does not exist,
% so the code that called this method would have had no parent signature to
% pass for the root anyway.
if nargin < 2
    parentSignature = '';
end

% Set current node's signature based on the parent node's signature, minus
% the parent node's type, plus the current node's own name and type, if
% appropriate for the type of node.  If the current node is the root node,
% set its signature to have no parent's signature included, since there is
% no parent.  Note, the parent signature is assumed to end with
% '.$Element$', so we remove '$Element$' from this signature, leaving just
% the '.' at the end.
if strcmp( parentSignature, '' )
    theStruct.Signature = '';
else
    theStruct.Signature = parentSignature( 1 : end - 9 );
end
if nodeIsTextNode( theStruct )
    theStruct.Signature = [ theStruct.Signature '$Text$' ];
elseif nodeIsAttributeNode( theStruct )
    theStruct.Signature = [ theStruct.Signature theStruct.Name ...
        '.$Attribute$' ];
elseif nodeIsElementNode( theStruct )
    theStruct.Signature = [ theStruct.Signature theStruct.Name ...
        '.$Element$' ];
    numChildren = nodeNumberOfChildren( theStruct );
    for i = 1 : numChildren
        % Create an "Signature" field for each child node so the next line
        % doesn't throw an error.
        theStruct.Children(i).Signature = '';
        theStruct.Children(i) = addNodeSignatures( ...
            theStruct.Children(i), ...
            theStruct.Signature    );
    end
else
    error( 'Invalid node type!' );
end
