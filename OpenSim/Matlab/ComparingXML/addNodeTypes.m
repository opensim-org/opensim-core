function theStruct = addNodeTypes( theStruct )
%
% Adds "Type" field to each node in theStruct, an XML DOM tree structure
% created by the parseXML method.
%
% Chand T. John, September 2007
% Stanford University
%

theStruct.Type = 'Null';
if nodeIsLeafNode( theStruct )
    if nodeIsTextNode( theStruct )
        theStruct.Type = 'Text';
    elseif nodeIsAttributeNode( theStruct )
        theStruct.Type = 'Attribute';
    else
        error( 'Found a leaf node that is not a text or attribute node!' );
    end
elseif nodeIsElementNode( theStruct )
    theStruct.Type = 'Element';
    numChildren = nodeNumberOfChildren( theStruct );
    for i = 1 : numChildren
        % Add a "Type" field to the current node so the next line doesn't
        % throw an error.
        theStruct.Children(i).Type = 'Null';
        theStruct.Children(i) = addNodeTypes( theStruct.Children(i) );
    end
else
    error( 'Failed to determine node type!' );
end
