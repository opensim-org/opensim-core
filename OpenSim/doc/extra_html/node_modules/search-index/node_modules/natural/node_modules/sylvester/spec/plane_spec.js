var sylvester = require('../lib/node-sylvester'),
Vector = sylvester.Vector,
Plane = sylvester.Plane;

describe('plane', function() {
    it('should create', function() {
	var plane = Plane.create([1,2,3], [5, 5, 5]);
	expect(plane.anchor).toEqual(Vector.create([1, 2, 3]));
    });

    it('should create with P$', function() {
	var A = Plane.create([1,2,3], [5, 5, 5]);
	var B = $P([1,2,3], [5, 5, 5]);

	expect(A).toEqual(B);
    });
});