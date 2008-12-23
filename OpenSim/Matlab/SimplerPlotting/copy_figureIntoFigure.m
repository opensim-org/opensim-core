function newAxes = copy_figureIntoFigure( oldFigureNumber, newFigureNumber, ...
    oldAxes, temporaryNewAxes )
%
% Compute affine transformation from old axes to temporary new axes.
%
% This function returns the two new axes within the destination figure.
%
% NOTE: the transformation is actually not a general affine transformation:
% it is M(x) = Dx + T, where D is a 2 x 2 diagonal matrix, x is a 2 x 1
% vector, and T is a 2 x 1 vector.  So basically it's a scaling followed by
% a translation.  If the positions of oldAxes and temporaryNewAxes,
% respectively, are:
% (  left  bottom width height )
% [   x1     y1     w1    h1   ] and
% [   x2     y2     w2    h2   ]
% the transformation is:
%  [ x ]   [ w2/w1   0   ][ x ]   [ x2 - (w2/w1)x1 ]
% M[   ] = [             ][   ] + [                ]
%  [ y ]   [   0   h2/h1 ][ y ]   [ y2 - (h2/h1)y1 ]
% So to transform an object with position [ x3 y3 w3 h3 ] from the old
% figure to the analogous position in the new figure, we need to set its
% position to be
% [ x4 y4 w4 h4 ] =
% [ x2 + (w2/w1)( x3 - x1 )
%   y2 + (h2/h1)( y3 - y1 )
%   (w2/w1)w3
%   (h2/h1)h3               ].
% 
oldPos = get(          oldAxes, 'Position' );
x1 = oldPos(1);
y1 = oldPos(2);
w1 = oldPos(3);
h1 = oldPos(4);
newPos = get( temporaryNewAxes, 'Position' );
x2 = newPos(1);
y2 = newPos(2);
w2 = newPos(3);
h2 = newPos(4);

% Copy and transform each child from old figure to new figure.
originalChildren = get( oldFigureNumber, 'Children' );
newAxes = originalChildren;
for j = 1 : length( originalChildren )
    oldChild = originalChildren(j);
    pos = get( oldChild, 'Position' );
    x3 = pos(1);
    y3 = pos(2);
    w3 = pos(3);
    h3 = pos(4);
    newChild = copyobj( oldChild, newFigureNumber );
    newChildPosition = [ ...
        x2 + (w2/w1)*( x3 - x1 ) ...
        y2 + (h2/h1)*( y3 - y1 ) ...
        (w2/w1)*w3 ...
        (h2/h1)*h3 ];
    set( newChild, 'Position', newChildPosition );
    newAxes(j) = newChild;
end
