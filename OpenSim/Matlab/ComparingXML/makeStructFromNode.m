function nodeStruct = makeStructFromNode( theNode )
%
% Creates structure of node info.
%
% From MATLAB example.
%
% Chand T. John, September 2007
% Stanford University
%

nodeStruct = struct(                         ...
   'Name'    , char( theNode.getNodeName ),  ...
   'Value'   , '',                           ...
   'Children', parseChildNodes( theNode ) );

if any( strcmp( methods( theNode ), 'getData' ) )
   nodeStruct.Value = char( theNode.getData ); 
else
   nodeStruct.Value = '';
end
