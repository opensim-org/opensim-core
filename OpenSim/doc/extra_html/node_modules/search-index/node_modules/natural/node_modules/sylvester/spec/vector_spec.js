
var sylvester = require('../lib/node-sylvester'),
Vector = sylvester.Vector;

var x = Vector.create([1, 2, 3]);

describe('vector', function() {
    it('should norm', function() {
	expect($V([1, 2, 3]).norm()).toBe(3.7416573867739413);
    });
    
    it('should log', function() {
	expect(x.log()).toEqual($V([0, 0.6931471805599453, 1.0986122886681098]));
    });

    it('should dot product', function() {
	expect(x.dot(Vector.create([2, 3, 4]))).toBe(20);
    });

    it('should support removal of head positions', function() {
	expect(x.chomp(1)).toEqual($V([2, 3]));
    });

    it('should sum', function() {
	expect(x.sum()).toBe(6);
    });

    it('should support addition of elements on the right side', function() {
	expect(x.augment([4, 5])).toEqual($V([1, 2, 3, 4, 5]));
    })

    it('should create with $V', function() {
	var a = $V([2, 3, 4]);
	var b = Vector.create([2, 3, 4]);
	expect(a).toEqual(b);
    });

    it('show allow for scalar addition', function() {
	var a = $V([2, 3, 4]);
	var b = a.add(1);
	expect(b.eql($V([3, 4, 5]))).toBeTruthy();
    });

    it('show add', function() {
	var a = $V([2, 3, 4]);
	var b = a.add($V([2, 4, 8]));
	expect(b.eql($V([, 7, 12]))).toBeTruthy();
    });
});
