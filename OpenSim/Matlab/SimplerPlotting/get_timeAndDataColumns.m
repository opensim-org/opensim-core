function timeAndDataColumns = get_timeAndDataColumns( ...
    sourceFiles, ...
    sourceColumnLabels, ...
    repeatedSourceColumnNumbers )

%
% Read in each file (don't read in repeated files more than once) and
% extract the desired columns from each file while it's open, accounting
% for repeated column labels using the third input argument to this
% function.
%

% Check that all arguments have the same length.
if length( sourceFiles ) ~= length( sourceColumnLabels )
    error( 'sourceFiles and sourceColumnLabels must have same length!' );
elseif length( sourceFiles ) ~= length( repeatedSourceColumnNumbers )
    error( 'repeatedSourceColumnNumbers must have same length as sourceFiles and sourceColumnLabels!' );
end

% Construct compact list of files to read in, so we can avoid reading in
% any one file more than once.
filesToRead = { sourceFiles{1} };
indicesInSourceColumnLabelsOfColumnsToReadFromEachFile = { 1 };
for i = 2 : length( sourceFiles )
    currentSourceFileIsNewFile = true;
    for j = 1 : i - 1
        if strcmpi( filesToRead{j}, sourceFiles{i} )
            currentSourceFileIsNewFile = false;
            indicesInSourceColumnLabelsOfColumnsToReadFromEachFile{j} = ...
                [ indicesInSourceColumnLabelsOfColumnsToReadFromEachFile{j} i ];
            break;
        end
    end
    if currentSourceFileIsNewFile
        j = length( filesToRead ) + 1;
        filesToRead{j} = sourceFiles{i};
        indicesInSourceColumnLabelsOfColumnsToReadFromEachFile{j} = i;
    end
end

% Read in each file one at a time, extracting the desired original data
% columns and time columns from each file.
originalTimeColumns = cell( 1, length( filesToRead ) );
originalDataColumns = cell( 1, length( sourceColumnLabels ) );
for j = 1 : length( filesToRead )
    q = read_motionFile( filesToRead{j} );
    originalTimeColumns{j} = q.data( :, 1 );
    indicesOfColumnsToReadInSourceColumnLabels = ...
        indicesInSourceColumnLabelsOfColumnsToReadFromEachFile{j};
    numberOfColumnsToRead = length( indicesOfColumnsToReadInSourceColumnLabels );
    for k = 1 : numberOfColumnsToRead
        indexOfColumnToReadInSourceColumnLabels = ...
            indicesOfColumnsToReadInSourceColumnLabels(k);
        matchingColumnLabelIndicesInQLabels = find( strcmpi( q.labels, ...
            sourceColumnLabels{ indexOfColumnToReadInSourceColumnLabels } ) );
        indexInQLabelsOfColumnToRead = ...
            matchingColumnLabelIndicesInQLabels( repeatedSourceColumnNumbers{j} );
        originalDataColumns{ indexOfColumnToReadInSourceColumnLabels } = ...
            q.data( :, indexInQLabelsOfColumnToRead );
    end
end

%
% Interpolate all time columns (and corresponding data columns) so that all
% data columns have the same length as the one common time column that is
% derived from all files.  Strategy: find the smallest dt for all time
% columns (average by (maxtime - mintime) / length(time)) and express all
% times in that time column using the interp1 function.  Pack the single
% time column and all data columns, in order of sourceColumnLabels or
% sourceFiles (they're the same order), and we're done!
%

% Find the finest average spacing of time values across all time columns,
% and the minimum and maximum time values during which all time columns
% overlap.
% We assume the time columns are all nondecreasing.
% We don't assume that the time values in each column are uniformly spaced,
% since in general, .sto files can have non-uniform time spacing.  But most
% of the time, even the spacing in .sto files is actually uniform.
timeCol = originalTimeColumns{1};
minAverageDt = ( timeCol( end ) - timeCol(1) ) / length( timeCol );
minTime = timeCol(1);
maxTime = timeCol( end );
for j = 2 : length( originalTimeColumns )
    timeCol = originalTimeColumns{j};
    averageDt = ( timeCol( end ) - timeCol(1) ) / length( timeCol );
    if averageDt < minAverageDt
        minAverageDt = averageDt;
    end
    if timeCol(1) > minTime
        minTime = timeCol(1);
    end
    if timeCol( end ) < maxTime
        maxTime = timeCol( end );
    end
end
if minAverageDt < 0
    error( 'Negative DT is not allowed!' );
end
if maxTime < minTime
    error( 'Min time is greater than max time!' );
end

% Express all data columns in a new time column with minimum time minTime,
% maximum time maxTime, and spacing minAverageDt.
commonTimeColumn = transpose( minTime : minAverageDt : maxTime );
newDataColumns = cell( 1, length( originalDataColumns ) );
for j = 1 : length( originalTimeColumns )
    indicesOfColumnsToInterpolateInOriginalDataColumns = ...
        indicesInSourceColumnLabelsOfColumnsToReadFromEachFile{j};
    numberOfColumnsToInterpolate = ...
        length( indicesOfColumnsToInterpolateInOriginalDataColumns );
    [ nonRepeatingTimeColumn, uniqueIndices, dummyVariable ] = ...
        unique( originalTimeColumns{j} );
    for k = 1 : numberOfColumnsToInterpolate
        indexOfColumnToInterpolateInOriginalDataColumns = ...
            indicesOfColumnsToInterpolateInOriginalDataColumns(k);
        repeatingDataColumn = ...
            originalDataColumns{ indexOfColumnToInterpolateInOriginalDataColumns };
        nonRepeatingDataColumn = repeatingDataColumn( uniqueIndices );
        newDataColumns{ indexOfColumnToInterpolateInOriginalDataColumns } = interp1( ...
            nonRepeatingTimeColumn, ...
            nonRepeatingDataColumn, ...
            commonTimeColumn );
    end
end

% Pack the common time column and all data columns into a single object for
% output from this function.
timeAndDataColumns = [ commonTimeColumn newDataColumns ];
