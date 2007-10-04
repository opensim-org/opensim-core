function theStruct = parseXML( filename )
%
% Converts an XML file to a MATLAB structure.
%
% Modified from MATLAB example.
%
% Chand T. John, September 2007
% Stanford University
%

% Read XML file into a Document Object Model (DOM) tree.
try
    tree = xmlread( filename );
catch
    error( 'Failed to read XML file %s.', filename );
end

% Recurse over child nodes. This could run into problems 
% with very deeply nested trees.
try
    aStruct = parseChildNodes( tree );
catch
    error( 'Unable to parse XML file %s.', filename );
end

% If the parsing resulted in multiple trees instead of a single tree,
% create a root node and make all trees into subtrees of that node.
try
    theStruct = addRootNodeIfNecessary( aStruct );
catch
    error( 'Unable to add root node to XML file %s.', filename );
end

% Create "Type" field for each node in the tree.
try
    theStruct = addNodeTypes( theStruct );
catch
    error( 'Unable to add node types to parsed XML file %s.', filename );
end
