
var sylvester = require('../lib/node-sylvester'),
Line = sylvester.Line,
Vector = sylvester.Vector;

describe('line', function() {
    it('should create', function() {
	var line = Line.create([1, 2], [5, 6]);
	expect(line.anchor).toEqual(Vector.create([1, 2, 0]));
    })

    it('should create with $L', function() {
	var a = Line.create([1, 2], [5, 6]);
	var b = $L([1, 2], [5, 6]);
	expect(a).toEqual(b);
    });
});
