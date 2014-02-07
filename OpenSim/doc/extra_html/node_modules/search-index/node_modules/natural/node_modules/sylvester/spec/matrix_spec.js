
var sylvester = require('../lib/node-sylvester'),
Matrix = sylvester.Matrix;
var A = Matrix.create([[1, 2, 3], [4, 5, 6]]);

describe('matrix', function() {
    it('should forward substitute', function() {
	var L = $M([
	    [1, 0, 0],
	    [0.5, 1, 0],
	    [2, 3, 1]
	]);

	var b = $V([1, 2, 3]);

	expect(L.forwardSubstitute(b).eql($V([1, 1.5, -3.5]))).toBeTruthy();
    });

    it('should back substitute', function() {
	var L = $M([
	    [4, 4],
	    [0, 1],
	]);

	var b = $V([1, 1.5]);

	expect(L.backSubstitute(b).eql($V([-1.25, 1.5]))).toBeTruthy();
    });

    it('should solve', function() {
	// 2x + 3y = 2
	// 4x + 4y = 1

	// x = -1.25 
	// y = 1.5
	var M = $M([
	    [2, 3],
	    [4, 4],
	]);

	var b = $V([2, 1]);
	expect(M.solve(b).eql($V([-1.25, 1.5])));
    });

    it('should partial pivot', function() {
        var B = $M([
            [3,  6,  -9],
            [-4, 1 , 10],
            [2,  5,  -3]
        ]);
	
	var P = Matrix.I(3);
	B.partialPivot(1, 1, P, B);
	
	expect(B.eql($M([
            [-4,  1 ,  10],
	    [ 3,  6,  -9],
	    [ 2,  5,  -3]
	]))).toBeTruthy();

	expect(P.eql($M([
	    [0, 1, 0], 
	    [1, 0, 0],
	    [0, 0, 1]
	]))).toBeTruthy();
    });

    describe('LU decomp', function() {
	it('should perform LU decomp on rectangular matrices', function() {
	    var D = $M([
		[3, 6],
		[2, 3],
		[4, 3],
		[2, 120],
	    ]);

	    var lu = D.luJs();
	    expect(lu.P.x((lu.L.x(lu.U))).eql(D)).toBeTruthy();

	    lu = D.luPack();
	    expect(lu.P.x((lu.L.x(lu.U))).eql(D)).toBeTruthy();
	});

	it('should perform LU decomp', function() {
	    var A = $M([
		[4,  2, 1,  4],
		[-9, 4, 3,  9],
		[11, 3, 11, 3],
		[-4, 5, 3,  1]
	    ]);
	    
	    var lu = A.lu();
	    
	    expect(lu.L.approxEql($M([
		[ 1, 0, 0, 0],
		[-0.818181818181818 , 1, 0, 0],
		[0.363636363636364, 0.140845070422535, 1, 0],
		[-0.363636363636364, 0.943661971830986, 0.921921921921922, 1]
	    ]))).toBeTruthy();
	    
	    expect(lu.U.approxEql($M([
		[11, 3, 11, 3],
		[ 0, 6.454545454545455, 12, 11.454545454545455],
		[ 0, 0, -4.690140845070422, 1.295774647887324],
		[ 0, 0, 0, -9.912912912912912]
	    ]))).toBeTruthy();
	    
	    expect(lu.P.eql($M([
		[0, 0, 1, 0],
		[0, 1, 0, 0],
		[1, 0, 0, 0],
		[0, 0, 0, 1]
	    ]))).toBeTruthy();
	});
	
	   it('should match LU JS to LAPACK', function() {
               var A = $M([
		   [4,  2, 1,  4],
		   [-9, 4, 3,  9],
		   [11, 3, 11, 3],
		   [-4, 5, 3,  1]
               ]);
	       
	       expect(A.luJs().U.approxEql(A.luPack().U)).toBeTruthy();
	       expect(A.luJs().L.approxEql(A.luPack().L)).toBeTruthy();
	   });
    });
    
    describe('PCA', function() {
	it('should PCA', function() {
	    var pca = $M([[1, 2], [5, 7]]).pcaProject(1);	    
	    
	    expect(pca.Z.approxEql($M([
		[-2.2120098720461616],
		[-8.601913944732665]
            ]))).toBeTruthy();

	    expect(pca.U.approxEql($M([[-0.5732529283807336, -0.819378471832714],
				 [-0.819378471832714, 0.5732529283807336]]))).toBeTruthy();

	});	

	it('should recover', function() {
            var U = $M([[-0.5732529283807336, -0.819378471832714],
			[-0.819378471832714, 0.5732529283807336]]);
            var Z = $M([[-2.2120098720461616],
			[-8.601913944732665]]);

            expect(Z.pcaRecover(U).eql($M([
		[1.268041136757554, 1.812473268636061],
		[4.931072358497068, 7.048223102871564]
            ]))).toBeTruthy();
	});
    });

    it('shoud triu', function () {
	var A2 = $M([
	    [ 1, -1,  2,  2],
	    [-1,  2,  1, -1],
	    [ 2,  1,  3,  2],
	    [ 2, -1,  2,  1]
	]);

	expect(A2.triu().eql($M([
	    [ 1, -1,  2,  2],
	    [ 0,  2,  1, -1],
	    [ 0,  0,  3,  2],
	    [ 0,  0,  0,  1]
	]))).toBeTruthy();

	expect(A2.triu(1).eql($M([
	    [ 0, -1,  2,  2],
	    [ 0,  0,  1, -1],
	    [ 0,  0,  0,  2],
	    [ 0,  0,  0,  0]
	]))).toBeTruthy();
    });

    it('should unroll', function() {
	expect(A.unroll().eql($V([1, 4, 2, 5, 3, 6]))).toBeTruthy();
    });

    it('should slice', function() {
	var A2 = $M([[1,2,3], [4,5,6], [7,8,9]]);
	var A3 = A2.slice(2, 3, 2, 3);
	expect(A3.eql($M([[5, 6], [8, 9]]))).toBeTruthy();
    });

    var U = $M([[-0.5110308651281587, 0.2132007163556105, 0.7071067811881557, 0.4397646068404634],
                [0.08729449334404742, -0.8528028654224428, 1.882731224298497e-12, 0.514885369921382],
                [-0.6856198518162525, -0.42640143271122105, -2.157344709257849e-12, -0.5900061329997158],
                [-0.5110308651281581, 0.21320071635561055, -0.7071067811849397, 0.4397646068456342]]);
    var S = $M([[5.85410196624969, 0, 0, 0],
                [0, 2.999999999999999, 0, 0],
                [0, 0, 1.0000000000000002, 0],
                [0, 0, 0, 0.8541019662496846]]);
    var V = $M([[-0.5110308651281575, 0.21320071635561047, -0.7071067811884307, -0.43976460684002194],
                [0.08729449334404744, -0.8528028654224414, -2.2043789591597237e-12, -0.5148853699213815],
                [-0.6856198518162527, -0.42640143271122066, 2.525858488366184e-12, 0.590006132999716],
                [-0.5110308651281579, 0.21320071635561044, 0.7071067811846652, -0.4397646068460757],
               ]);

    var ASVD = $M([
	[ 1, -1, 2,  2],
	[-1,  2, 1, -1],
	[ 2,  1, 3,  2],
	[ 2, -1, 2,  1]
    ]);
    
    it('should svd', function() {
	var svd = ASVD.svdJs();	
	expect(svd.U.eql(U)).toBeTruthy();
	expect(svd.S.eql(S)).toBeTruthy();
	expect(svd.V.eql(V)).toBeTruthy();
    });

    it('should have matching svds for js and lapack', function() {
	var svdJs = ASVD.svdJs();	
	var svdPack = ASVD.svdPack();

	expect(svdJs.U.eql(svdPack.U)).toBeTruthy();
	expect(svdJs.S.eql(svdPack.S)).toBeTruthy();
	expect(svdJs.V.eql(svdPack.V)).toBeTruthy();
    });

    var QRin = $M([
        [1, -1, 2, 2],
        [-1, 2, 1, -1],
        [2, 1, 3, 2],
        [2, -1, 2, 1]
    ]);

    var Qout = $M([[-0.316227766016838, 0.28342171556262064, 0.8226876614429064, -0.3779644730092273],
                   [0.31622776601683794, -0.6883098806520787, 0.5323273103454103, 0.3779644730092272],
                   [-0.6324555320336759, -0.6478210641431328, -0.19357356739833098, -0.37796447300922714],
                   [-0.6324555320336759, 0.16195526603578317, 0.048393391849582745, 0.7559289460184544]]);
    var Rout = $M([[-3.1622776601683795, 0.9486832980505139, -3.478505426185217, -2.8460498941515415],
                   [1.91055907392895e-17, -2.4698178070456938, -1.7410191098846692, 0.1214664495268375],
                   [-2.254600901479451e-16, 2.0686390257580927e-16, 1.6937687147353957, 0.7742942695933234],
                   [3.446764628337833e-17, 8.098938594673387e-17, 2.220446049250313e-16, -1.1338934190276815]]);

    it('should qr from javascript', function() {
	var qr = QRin.qrJs();
	expect(qr.Q.eql(Qout)).toBeTruthy();
	expect(qr.R.eql(Rout)).toBeTruthy();
    });

    it('should qr from lapack', function() {
	var qr = QRin.qrPack();
	expect(qr.Q.eql(Qout)).toBeTruthy();
	expect(qr.R.eql(Rout)).toBeTruthy();
    });

    it('should create a 1\'s matrix', function() {
	var Ones = Matrix.One(2, 3);
	expect(Ones.eql($M([[1,1,1], [1,1,1]]))).toBeTruthy();
    });

    it('columns should be retrievable as vectors', function() {
	expect(A.column(2).eql($V([2, 5]))).toBeTruthy();
    });

    it('should log', function() {
	expect(A.log().eql($M([[0, 0.6931471805599453, 1.0986122886681098],
			       [1.3862943611198906, 1.6094379124341003, 1.791759469228055]]))).toBeTruthy();
    });

    it('should sum', function() {
	expect(A.sum()).toBe(21);
    });

    it('should multiply', function() {
	expect(A.x(Matrix.create([[1, 2], [3, 4], [5, 6]])).eql(Matrix.create([[22, 28], [49, 64]]))).toBeTruthy();
    });

    it('should multiply', function() {
	var B = $M([[1, 2, 3], [4, 5, 6]]);
	expect(A.eql(B)).toBeTruthy();
    });

    it('should evaluate equal matrices', function() {
	var A = $M([[1, 2, 3], [4, 5, 6]]);
	var B = $M([[1, 2, 3], [4, 5, 6]]);

	expect(A.eql(B)).toBeTruthy();
    });

    it('should evaluate inequal matrices', function() {
	var A = $M([[1, 2, 3], [4, 5, 6]]);
	var B = $M([[1, 2, 3], [4, 5, 7]]);

	expect(A.eql(B)).toBeFalsy();
    });

    it('should snap', function() {
	expect($M([[1, 1.1, 1.00000001], [4, 5, 6]]).snapTo(1).eql(
	    $M([[1, 1.1, 1], [4, 5, 6]]))).toBeTruthy();
    });

    it('should compute the minimum index of matrix rows', function() {
	expect($M([[1, 2, 3], [2, 1, 3], [2, 1, 0]]).minColumnIndexes().eql($V([2, 1, 3])));
    });

    it('should compute the minimum value of matrix rows', function() {
	expect($M([[1, 2, 3], [2, 1, 3], [2, 1, 0]]).minColumns().eql($V([1, 1, 0])));
    });

    it('should compute the maximum index of matrix rows', function() {
	expect($M([[1, 2, 3], [2, 3, 2], [2, 1, 0]]).maxColumnIndexes().eql($V([3, 2, 1])));
    });

    it('should compute the maximum value of matrix rows', function() {
	expect($M([[1, 2, 3], [2, 1, 3], [2, 1, 0]]).maxColumns().eql($V([3, 3, 2])));
    });
});
