function findAndFillInFirstAttributesOfTags( fileNode, desiredTagNames, values )

numberOfChildrenInMainBody = fileNode.getChildNodes.getLength;
for i = 0:numberOfChildrenInMainBody-1
    currentNode = fileNode.getChildNodes.item(i);
    if currentNode.hasAttributes
        currentTagName = char( currentNode.getTagName );
        anyDesiredTagNamesThatMatchCurrentTag = strcmpi( desiredTagNames, currentTagName );
        indicesOfCurrentTagInDesiredTagNames = find( anyDesiredTagNamesThatMatchCurrentTag );
        if ~isempty( indicesOfCurrentTagInDesiredTagNames )
            if length( indicesOfCurrentTagInDesiredTagNames ) > 1
                error( ['Repeated occurrences of tag ' currentTagName ' in desiredTagNames.'] );
            end
            indexOfCurrentTagInDesiredTagNames = indicesOfCurrentTagInDesiredTagNames(1);
            currentNode.getAttributes.item(0).setValue( values{ indexOfCurrentTagInDesiredTagNames } );
        end
    end
end
