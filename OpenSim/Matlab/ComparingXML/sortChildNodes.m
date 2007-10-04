function tree = sortChildNodes( tree )
%
% Sort all children of each node in the tree.  Sorting is done based on
% Type, then Name, and then Value.
%
% Chand T. John, September 2007
% Stanford University
%

% Extract Type, Name, and Value fields from each child node, and sort each
% child node.
numChildren = nodeNumberOfChildren( tree );
if numChildren < 1
    % If there are no children, do nothing!
    return;
end
childTypes  = cell( 1, numChildren );
childNames  = cell( 1, numChildren );
childValues = cell( 1, numChildren );
for i = 1 : numChildren
    child = tree.Children(i);
    childTypes{i}  = child.Type;
    childNames{i}  = child.Name;
    childValues{i} = child.Value;
    tree.Children(i) = sortChildNodes( child );
end

% Sort children by Type.
[ childTypes, I ] = sort( childTypes );
tree.Children( 1 : numChildren ) = tree.Children(I);
childNames( 1 : numChildren ) = childNames(I);
childValues( 1 : numChildren ) = childValues(I);

% Find groups of indices of children that share the same Type.
typeClasses = {};
currentType = '';
currentClass = 0;
for i = 1 : numChildren
    child = tree.Children(i);
    typeAlreadyFound = strcmp( child.Type, currentType );
    if typeAlreadyFound
        % Add this child's index to the current type class index list.
        typeClasses{ currentClass } = [ typeClasses{ currentClass } i ];
    else
        % End search for previous type, change currentType to current
        % child's type, and add this child's index to a new type class
        % index list.
        currentType = child.Type;
        typeClasses{ currentClass + 1 } = i;
        currentClass = currentClass + 1;
    end
end
% Now typeClasses is a cell array in which each cell contains an array of
% indices of children who all share the same Type.

% Sort each set of children with the same Type, by Name.  Our strategy is
% to iterate through typeClasses, and sort the children in tree.Children
% who share the same Type, by Name.  These names are stored in childNames,
% in the same order as the children are ordered in tree.Children.  So we
% can sort childNames, keep track of how the elements were sorted in the
% process, and then appropriately reorder the children whose names were
% just sorted, and sort the corresponding values in childValues as well.
for i = 1 : length( typeClasses )
    indicesToSortByName = typeClasses{i};
    names = childNames( indicesToSortByName );
    [ names, I ] = sort( names );
    % Transform I to indicesToSortyByName: since 1:length(names) was the
    % original I before sorting, and the indices of the elements with
    % indices 1:length(names) in names had indices indicesToSortByName in
    % childNames, we can use the mapping
    % 1:length(names) |--> indicesToSortByName(1:length(names))
    % to determine how the new permutation of 1:length(names), I, should be
    % transformed back to a permutation of indicesToSortByName.  In fact,
    % MATLAB gives us an easy way to deal with permutations: we just want
    % to reorder the elements in indicesToSortByName appropriately:
    newIndicesSortedByName = indicesToSortByName(I);
    % Now, reorder the childNames array, children, and childValues array,
    % replacing the elements with indices indicesToSortByName in their
    % reordered form, with indices newIndicesSortedByName.
    % Note: one more intuitive way to think about the statements below is,
    % for example:
    % childNames( indicesToSortByName(1:length(indicesToSortByName)) ) =
    %    childNames( indicesToSortByName(I) );
    % which is analogous to
    % indicesToSortByName( 1:length(indicesToSortByName ) =
    %    indicesToSortByName(I);
    % which is the typical way we reorder elements after running "sort".
    childNames( indicesToSortByName ) = ...
        childNames( newIndicesSortedByName );
    tree.Children( indicesToSortByName ) = ...
        tree.Children( newIndicesSortedByName );
    childValues( indicesToSortByName ) = ...
        childValues( newIndicesSortedByName );
end
% The children and childNames and childValues arrays are now ordered by
% Type and Name.

% Find groups of indices of children that share the same Type and Name.
% Our strategy is to iterate through typeClasses, and for each list
% typeClasses{i}, we will loop through its indices, and for each index j,
% we will check the name of tree.Children(indices(j)) to see if it is a new
% name, or an old name, and add it to a new or old index list in
% nameClasses, respectively.  Each time we get to a new array in
% typeClasses, i.e. each time we increment i, we will reset the currentName
% to '' so that a new class is created for each Type as well as each Name.
nameClasses = {};
currentClass = 0;
for i = 1 : length( typeClasses )
    currentName = '';
    indices = typeClasses{i};
    for j = 1 : length( indices )
        childIndex = indices(j);
        child = tree.Children( childIndex );
        nameAlreadyFound = strcmp( child.Name, currentName );
        if nameAlreadyFound
            % Add this child's index to the current name class index list.
            nameClasses{ currentClass } = ...
                [ nameClasses{ currentClass } childIndex ];
        else
            % End search for previous name, change currentName to current
            % child's name, and add this child's index to a new name class
            % index list.
            currentName = child.Name;
            if j == 1 && i > 1
                % currentClass was already incremented previously, unless
                % i == 1, in which case currentClasses is still zero, and
                % the "else" statement should be executed instead.
                nameClasses{ currentClass } = childIndex;
            else
                nameClasses{ currentClass + 1 } = childIndex;
                currentClass = currentClass + 1;
            end
        end
    end
    currentClass = currentClass + 1;
end
% Now nameClasses is a cell array in which each cell contains an array of
% indices of children who all share the same Type and Name.

% Sort each set of children with the same Type and Name, by Value.  Our
% strategy is the same as before, but using nameClasses instead of
% typeClasses.
for i = 1 : length( nameClasses )
    indicesToSortByValue = nameClasses{i};
    values = childValues( indicesToSortByValue );
    [ values, I ] = sort( values );
    newIndicesSortedByValue = indicesToSortByValue(I);
    tree.Children( indicesToSortByValue ) = ...
        tree.Children( newIndicesSortedByValue );
end
% The children and childNames and childValues arrays are now ordered by
% Type, Name, and Value.
