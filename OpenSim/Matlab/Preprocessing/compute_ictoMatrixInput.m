function ictoIn = compute_ictoMatrixInput(motFileName)
% AUTHOR: Chand T. John
% We assume the right limb is on force plate #1 and
% the left limb is on force plate #2.  This will always
% be the case for the initial Delaware data collected in
% Jill Higginson's lab on June 30, 2006 by Chand and
% Jill and her students; as long as the subject was
% facing forward during the motion trial, and had one
% leg on each belt of the treadmill.  We also assume no
% flight phase during the entire trial, i.e. at least
% one leg is in contact with the ground at any time.
% Other assumptions:
% - first column in motion file is time column
% - first ground_force_vy column is for right foot, and
%   its value is zero when the right foot is not in contact
%   with the ground
% - second ground_force_vy column is for left foot, and
%   its value is zero when the right foot is not in contact
%   with the ground

% Read the motion file.
% q will have four fields:
% q.nr = number of rows in the motion file
% q.nc = number of columns in the motion file
% q.labels = row vector of column labels in the motion file
% q.data = array of data values in the motion file, with
%          the first column being the time column and later
%          columns corresponding to their column labels
q = read_motionFile(motFileName);

% Extract the array of times from the motion file.
timeColumn = q.data(:,1);

% Compute force plate frame number for each time in
% timeColumn.
frameNumbers = timeColumn * 600;

% Find the indices of all occurrences of ground_force_vy
% as a column label.
groundForceVyColumnNumbers = find(strcmp(q.labels,'ground_force_vy'));
% The row vector groundForceVyColumnNumbers contains a list
% of all indices in q.labels where 'ground_force_vy' occurs.

% It is generally assumed that there will be exactly two
% columns with label 'ground_force_vy': one for the right
% foot and one for the left foot.  Here, if there are less
% than two such columns, we print an error message and exit
% this program.  If there are more than two such columns,
% we will pay attention to the first two columns and ignore
% all later columns.
numberOfGroundForceVyColumns = length(groundForceVyColumnNumbers);
if numberOfGroundForceVyColumns < 2
    sprintf('----ERROR---- There are fewer than two columns with the label \''ground_force_vy\'' in the motion file %s.  I am giving up now.', motFileName);
    return;
else
    if numberOfGroundForceVyColumns > 2
        sprintf('---WARNING--- There are more than two columns with the label \''ground_force_vy\'' in the motion file %s.  I will simply use the first two columns and ignore all later columns.', motFileName);
    end
end
% We can now safely assume that there are at least two
% occurrences of ground_force_vy in q.labels, otherwise
% the program would have terminated by now.  We will simply
% use the first two columns indicated in the row vector
% groundForceVyColumnNumbers.

% Extract the two ground_force_vy columns from the motion file.
rightGroundForceY = q.data(:,groundForceVyColumnNumbers(1));
leftGroundForceY  = q.data(:,groundForceVyColumnNumbers(2));

% Compute the intervals during which the right foot was in contact
% with the ground.  The resulting array will look like:
% rightContactRanges = [ <1st contact start>  <1st contact end>
%                        <2nd contact start>  <2nd contact end>
%                                         .
%                                         .
%                                         .
%                        <last contact start> <last contact end> ]
% Note, <1st contact start> is zero if the right foot was already
% in contact with the ground at the beginning of the motion.  Also,
% if the right foot remains in contact with the ground when the
% motion ends, then <last contact end> is set to zero.

% The variable rightFootIsInContact is a flag that is true when
% the right foot is in contact with the ground, and false
% otherwise.
rightFootIsInContact = (rightGroundForceY(1) ~= 0);
% Our strategy is to look at each element of the rightGroundForceY
% array from the first element to the last element.  When looking
% at each element of the array, we keep in mind whether or not the
% right foot was already in contact with the ground (i.e. whether
% the previous value in the array was nonzero).  If the right foot
% was in contact with the ground and then we encounter a zero, all
% of a sudden we realize that the right foot is no longer touching
% the ground, since the force plate now has zero force on it.  If
% the right foot was not in contact and suddenly we see a nonzero
% element in the array, then we realize that the right foot has now
% gone into contact at that instant in time.  Each time we realize
% that the right foot has gone into contact, we set the variable
% contactStartFrameNumber to the number of the current frame.  Each
% time we realize that the right foot has gone out of contact, we
% set the variable contactEndFrameNumber to the number of the
% current frame, append the interval
% [contactStartFrameNumber, contactEndFrameNumber]
% to the rightContactRanges matrix, and then set both
% contactStartFrameNumber and contactEndFrameNumber to -1.  Then
% we continue reading elements of the rightGroundForceY array one
% by one until the right foot goes into contact again, and we
% repeat the process until we have looked at every element in the
% array.
contactStartFrameNumber = -1;
contactEndFrameNumber = -1;
% Initially, let rightContactRanges array be a 2 x 1 array with a
% dummy row, [-1 -1], in it.  This is purely for coding purposes;
% these two -1's have no logical meaning whatsoever.
rightContactRanges = [-1 -1];
% If the right foot is already in contact with the ground, then
% set the contactStartFrameNumber to zero (even though the actual
% first frame of the motion has frame number 1).  This is just our
% convention for indicating that the foot came into contact with
% the ground before the motion even started.
if rightFootIsInContact
    contactStartFrameNumber = 0;
end
for i = 2:length(rightGroundForceY)
    if rightGroundForceY(i) == 0 & rightFootIsInContact
        % The right foot WAS in contact with the ground.  But oh,
        % look!  Suddenly the force plate is recording no force,
        % which means the right foot has JUST left the ground.
        % So let's make a note of that by setting the flag
        % rightFootIsInContact to false, and appending the
        % frame numbers for the start and end of contact to the
        % rightContactRanges array.  Then let's reset
        % contactStartFrameNumber and contactEndFrameNumber to
        % -1, and then allow this for loop to step through more
        % elements of the rightGroundForceY array until the
        % right foot touches the ground again.
        rightFootIsInContact = false;
        contactEndFrameNumber = frameNumbers(i-1);
        rightContactRanges = [rightContactRanges; ...
                              contactStartFrameNumber, ...
                              contactEndFrameNumber];
        contactStartFrameNumber = -1;
        contactEndFrameNumber = -1;
    else
        if rightGroundForceY(i) ~= 0 & ~rightFootIsInContact
            % The right foot wasn't in contact with the ground
            % so far, but oh, look!  The force plate suddenly
            % has a nonzero value, which means the right foot
            % has JUST come into contact with the ground.  Let's
            % set contactStartFrameNumber to the number of the
            % current frame, and then let this for loop continue
            % to step through elements of the rightGroundForceY
            % array until the right foot leaves the ground.
            rightFootIsInContact = true;
            contactStartFrameNumber = frameNumbers(i);
        end
    end
end
% If the right foot was still in contact with the ground when
% the motion ended, i.e. if rightFootIsInContact == true, then
% add [contactStartFrame 0] to the rightContactRanges array.
if rightFootIsInContact
    rightContactRanges = [rightContactRanges; ...
                          contactStartFrameNumber, 0];
end
% Remove the dummy [-1 -1] row from the beginning of the
% rightContactRanges array.
rightContactRanges = rightContactRanges(2:end,:);

% Repeat the above for the left foot, i.e. compute the intervals
% during which the left foot was in contact with the ground.
leftFootIsInContact = (leftGroundForceY(1) ~= 0);
contactStartFrameNumber = -1;
contactEndFrameNumber = -1;
% Initially, let leftContactRanges array be a 2 x 1 array with a
% dummy row, [-1 -1], in it.  This is purely for coding purposes;
% these two -1's have no logical meaning whatsoever.
leftContactRanges = [-1 -1];
% If the left foot is already in contact with the ground, then
% set the contactStartFrameNumber to zero (even though the actual
% first frame of the motion has frame number 1).  This is just our
% convention for indicating that the foot came into contact with
% the ground before the motion even started.
if leftFootIsInContact
    contactStartFrameNumber = 0;
end
for i = 2:length(leftGroundForceY)
    if leftGroundForceY(i) == 0 & leftFootIsInContact
        % The left foot WAS in contact with the ground.  But oh,
        % look!  Suddenly the force plate is recording no force,
        % which means the left foot has JUST left the ground.
        % So let's make a note of that by setting the flag
        % leftFootIsInContact to false, and appending the
        % frame numbers for the start and end of contact to the
        % leftContactRanges array.  Then let's reset
        % contactStartFrameNumber and contactEndFrameNumber to
        % -1, and then allow this for loop to step through more
        % elements of the leftGroundForceY array until the
        % left foot touches the ground again.
        leftFootIsInContact = false;
        contactEndFrameNumber = frameNumbers(i-1);
        leftContactRanges = [leftContactRanges; ...
                              contactStartFrameNumber, ...
                              contactEndFrameNumber];
        contactStartFrameNumber = -1;
        contactEndFrameNumber = -1;
    else
        if leftGroundForceY(i) ~= 0 & ~leftFootIsInContact
            % The left foot wasn't in contact with the ground
            % so far, but oh, look!  The force plate suddenly
            % has a nonzero value, which means the left foot
            % has JUST come into contact with the ground.  Let's
            % set contactStartFrameNumber to the number of the
            % current frame, and then let this for loop continue
            % to step through elements of the leftGroundForceY
            % array until the left foot leaves the ground.
            leftFootIsInContact = true;
            contactStartFrameNumber = frameNumbers(i);
        end
    end
end
% If the left foot was still in contact with the ground when
% the motion ended, i.e. if leftFootIsInContact == true, then
% add [contactStartFrame 0] to the leftContactRanges array.
if leftFootIsInContact
    leftContactRanges = [leftContactRanges; ...
                         contactStartFrameNumber, 0];
end
% Remove the dummy [-1 -1] row from the beginning of the
% leftContactRanges array.
leftContactRanges = leftContactRanges(2:end,:);

% Now, the rightContactRanges and leftContactRanges arrays
% contain the start and end frame numbers for each time
% interval during which a foot was in contact with the
% ground.  We will now perform some sanity checks and
% determine which leg was the first to come in contact
% with the ground.
rightDimensions = size( rightContactRanges );
leftDimensions = size( leftContactRanges );
rightNumRows = rightDimensions(1);
leftNumRows = leftDimensions(1);
rightFirstPositiveContactStartFrameNumber = -1;
leftFirstPositiveContactStartFrameNumber = -1;
% Find frame number of first initial contact of the right limb, if such an
% initial contact exists.
for i = 1:rightNumRows
    if rightContactRanges(i,1) > 0
        rightFirstPositiveContactStartFrameNumber = rightContactRanges(i,1);
        break;
    end
end
% Find frame number of first initial contact of the left limb, if such an
% initial contact exists.
for i = 1:leftNumRows
    if leftContactRanges(i,1) > 0
        leftFirstPositiveContactStartFrameNumber = leftContactRanges(i,1);
        break;
    end
end
% Now we can assume that right/left FirstPositiveContactStartFrameNumber
% variables are each either -1 or a positive number.  Just to be sure,
% let's throw an error if this is not the case.
if rightFirstPositiveContactStartFrameNumber ~= -1 && rightFirstPositiveContactStartFrameNumber <= 0
    error( 'rightFirstPositiveContactStartFrameNumber should be -1 or positive!' );
end
if leftFirstPositiveContactStartFrameNumber ~= -1 && leftFirstPositiveContactStartFrameNumber <= 0
    error( 'leftFirstPositiveContactStartFrameNumber should be -1 or positive!' );
end
% Assign the first initial contact limb to be the one with the lower
% positive contact start frame number.  If only one limb has such a frame
% number, then the first initial contact limb is that limb.  If neither
% limb has such a frame number, then no limb ever comes into contact with a
% force plate, so throw an error.
if rightFirstPositiveContactStartFrameNumber == -1
    if leftFirstPositiveContactStartFrameNumber == -1
        error( 'Neither limb has a first initial contact with the ground!' );
    else
        ictoIn.firstICLimb = 'L';
    end
elseif leftFirstPositiveContactStartFrameNumber == -1
    ictoIn.firstICLimb = 'R';
elseif rightFirstPositiveContactStartFrame < leftFirstPositiveContactStartFrame
    ictoIn.firstICLimb = 'R';
else
    ictoIn.firstICLimb = 'L';
end

% If we got this far, we have now decided, for sure, which
% leg came in contact with the ground first.  If the right
% leg came in contact first, then force plate #1 was the first
% force plate to be hit by the subject.  Otherwise, the
% first force plate to be hit by the subject was force
% plate #2.
if ictoIn.firstICLimb == 'R'
    ictoIn.firstICFP = 1;
else
    % Here we assume that ictoIn.firstICLimb == 'L'.
    ictoIn.firstICFP = 2;
end

% We now know which leg came in contact with the ground
% first, as well as the frame number ranges during which
% each leg was in contact with the ground.  So, starting
% with the first leg that was in contact with the ground,
% we should be able to just fill in the rows of the
% contactRanges matrix by alternating the starting leg's
% contact ranges and the other leg's contact ranges.
% This assumes that the subject was performing a normal
% walking motion, i.e. the following sequence of contact
% events repeats:
% 1. One leg hits the ground, at which point both legs
%    are in contact with the ground.
% 2. The other leg leaves the ground, leaving only the
%    first leg in contact with the ground.
% 3. The other leg hits the ground, at which point both
%    legs are in contact with the ground.
% 4. The first leg leaves the ground, leaving only the
%    other leg in contact with the ground.
% 5. The first leg hits the ground, at which point we're
%    back to step 1, and the cycle repeats.
% These steps repeat until the motion is over.  Note that
% we assume that the motion can start at any step in this
% process and cycle through the steps in the above order.
% I.e., it is not necessary for the motion to start at
% step 1 and then go to step 2, 3, 4, etc.  It is okay for
% the motion to start at step 3, and then go to 4, 5, 1,
% etc.  But the motion must cycle through these steps in
% the order above, regardless of the step in which it
% the motion actually starts.
%
% If the right foot was the first one to touch the ground,
% we want the following output:
% ictoIn.contactRanges = [rightContactRanges(1,:);
%                         leftContactRanges(1,:);
%                         rightContactRanges(2,:);
%                         leftContactRanges(2,:);
%                                   .
%                                   .
%                                   .
%                         rightContactRanges(end,:)]
% If the left foot was the first one to touch the ground,
% we would want the analogous output with 'left' and 'right'
% switched.
%
% However, I will apply a slightly more robust approach:
% I will iterate through the rows of both arrays, and at
% each step, I will choose the interval with the earlier
% starting frame number (and if they're the same, I'll
% choose the interval with the earlier ending frame number)
% and I will add that interval to ictoIn.contactRanges.
% If the ranges aren't alternating between right and left
% foot contacts, I will still continue adding intervals to
% contactRanges until it's done.
ictoIn.contactRanges = [rightContactRanges; leftContactRanges];
ictoIn.contactRanges = sortrows(ictoIn.contactRanges);

% If there is more than one partial event in the beginning
% of the motion, i.e. if both feet were touching the ground
% at the beginning of the motion, then only record the last
% such partial event, i.e. only have one row in the array
% with a 0 as its first element.  To be more robust, in case
% for some reason there are more than two rows with zero as
% the first element, I will keep removing rows until there
% is only one row left at the beginning with zero as its
% first element.
while ictoIn.contactRanges(2,1) == 0
    ictoIn.contactRanges = ictoIn.contactRanges(2:end,:);
end

% Analogously, have only one partial event represented
% at the end of the array.
while ictoIn.contactRanges(end-1,2) == 0
    ictoIn.contactRanges = ictoIn.contactRanges(1:end-1,:);
end
% This should be the final output.

% ictoIn.contactRanges is currently a double array.
% Convert it to an integer array.
ictoIn.contactRanges = floor(ictoIn.contactRanges);