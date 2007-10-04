function children = parseChildNodes( theNode )
%
% Recurses over node children.
%
% Modified from MATLAB example to have separate "Attribute" nodes, and
% only "Name", "Value", and "Children" fields for each node with no
% "Attributes" field.  Basically, there will be three types of nodes:
% 1. Element nodes, which are non-leaf nodes that have only a Name field
% 2. Text nodes, which are leaf nodes that have only a Value field
% 3. Attribute nodes, which are leaf nodes that have both Name and Value
%    fields
%
% Chand T. John, September 2007
% Stanford University
%

children = [];
if theNode.hasChildNodes
   childNodes = theNode.getChildNodes;
   numChildNodes = childNodes.getLength;

   numAttributes = 0;
   if theNode.hasAttributes
       attributes = theNode.getAttributes;
       numAttributes = attributes.getLength;
   end
   allocCell = cell( 1, numChildNodes + numAttributes );

   children = struct(                            ...
      'Name',     allocCell, 'Value', allocCell, ...
      'Children', allocCell );

    for count = 1 : numChildNodes
        theChild = childNodes.item( count - 1 );
        children( count ) = makeStructFromNode( theChild );
    end
    for count = numChildNodes + 1 : numChildNodes + numAttributes
        attributeNumber = count - numChildNodes - 1;
        theAttribute = attributes.item( attributeNumber );
        children( count ).Name     = char( theAttribute.getName );
        children( count ).Value    = char( theAttribute.getValue );
        children( count ).Children = [];
    end
elseif theNode.hasAttributes % has attributes but no children
    numChildNodes = 0;
    attributes = theNode.getAttributes;
    numAttributes = attributes.getLength;
    allocCell = cell( 1, numAttributes );

    children = struct(                            ...
       'Name',     allocCell, 'Value', allocCell, ...
       'Children', allocCell );

    for count = 1 : numAttributes
        attributeNumber = count - numChildNodes - 1;
        theAttribute = attributes.item( attributeNumber );
        children( count ).Name     = char( theAttribute.getName );
        children( count ).Value    = char( theAttribute.getValue );
        children( count ).Children = [];
    end
end
