
var sylvester = require('../lib/node-sylvester'),
Line = sylvester.Line,
LineSegment = sylvester.Line.Segment,
Vector = sylvester.Vector;

describe('line segment', function() {
    it('should create', function() {
	var lineSegment = Line.Segment.create([1, 2], [5, 6]);
	expect(lineSegment.line.anchor).toEqual(Vector.create([1, 2, 0]));
    })
});
