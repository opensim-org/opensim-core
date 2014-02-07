/*
Copyright (c) 2011, Chris Umbel

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

var BayesClassifier = new require('../lib/apparatus/classifier/bayes_classifier');

describe('bayes', function() {    
    it('should perform binary classifcation', function() {
        var bayes = new BayesClassifier();
        bayes.addExample([1,1,1,0,0,0], 'one');
        bayes.addExample([1,0,1,0,0,0], 'one');
        bayes.addExample([1,1,1,0,0,0], 'one');
        bayes.addExample([0,0,0,1,1,1], 'two');
        bayes.addExample([0,0,0,1,0,1], 'two');
        bayes.addExample([0,0,0,1,1,0], 'two');

        bayes.train();
        
        expect(bayes.classify([1,1,0,0,0,0])).toBe('one');
        expect(bayes.classify([0,0,0,0,1,1])).toBe('two');
    });

    it('should classify', function() {
        var bayes = new BayesClassifier();
        bayes.addExample([1,1,1,0,0,0,0,0,0], 'one');
        bayes.addExample([1,0,1,0,0,0,0,0,0], 'one');
        bayes.addExample([1,1,1,0,0,0,0,0,0], 'one');
        bayes.addExample([0,0,0,1,1,1,0,0,0], 'two');
        bayes.addExample([0,0,0,1,0,1,0,0,0], 'two');
        bayes.addExample([0,0,0,1,1,0,0,0,0], 'two');
        bayes.addExample([0,0,0,0,0,0,1,1,1], 'three');
        bayes.addExample([0,0,0,0,0,0,1,0,1], 'three');
        bayes.addExample([0,0,0,0,0,0,1,1,0], 'three');

        bayes.train();
        
        expect(bayes.classify([1,1,0,0,0,0,1,0,0])).toBe('one');
        expect(bayes.classify([0,0,1,1,1,0,0,0,1])).toBe('two');
        expect(bayes.classify([1,0,0,0,1,0,0,1,1])).toBe('three');

    });

    it('should classify with deserialized classifier', function() {
        var bayes = new BayesClassifier();
        bayes.addExample([1,1,1,0,0,0,0,0,0], 'one');
        bayes.addExample([1,0,1,0,0,0,0,0,0], 'one');
        bayes.addExample([1,1,1,0,0,0,0,0,0], 'one');
        bayes.addExample([0,0,0,1,1,1,0,0,0], 'two');
        bayes.addExample([0,0,0,1,0,1,0,0,0], 'two');
        bayes.addExample([0,0,0,1,1,0,0,0,0], 'two');
        bayes.addExample([0,0,0,0,0,0,1,1,1], 'three');
        bayes.addExample([0,0,0,0,0,0,1,0,1], 'three');
        bayes.addExample([0,0,0,0,0,0,1,1,0], 'three');

        bayes.train();

    	var obj = JSON.stringify(bayes);
    	var newBayes = BayesClassifier.restore(JSON.parse(obj));
        
        expect(newBayes.classify([1,1,0,0,0,0,1,0,0])).toBe('one');
        expect(newBayes.classify([0,0,1,1,1,0,0,0,1])).toBe('two');
        expect(newBayes.classify([1,0,0,0,1,0,0,1,1])).toBe('three');
    });

    it('should classify with smoothing', function() {
        var bayes = new BayesClassifier(0.3);
        bayes.addExample([1,1,1,0,0,0,0,0,0], 'one');
        bayes.addExample([0,0,1,0,0,0,0,0,0], 'one');
        bayes.addExample([0,0,1,0,0,0,0,0,0], 'one');
        bayes.addExample([0,0,0,1,1,1,0,0,0], 'two');
        bayes.addExample([0,0,0,0,0,1,0,0,0], 'two');
        bayes.addExample([0,0,0,0,1,0,0,0,0], 'two');
        bayes.addExample([0,0,0,0,0,0,1,1,1], 'three');
        bayes.addExample([0,0,0,0,0,0,0,0,1], 'three');
        bayes.addExample([0,0,0,0,0,0,0,1,0], 'three');

        bayes.train();

        expect(bayes.classify([1,0,0,0,0,0,1,0,0])).toBe('one');
        expect(bayes.classify([0,0,1,1,1,0,0,0,1])).toBe('two');
        expect(bayes.classify([1,0,0,0,1,0,0,1,1])).toBe('three');
    });  

    it('should classify with sparse observations', function() {
        var bayes = new BayesClassifier();
        bayes.addExample({'a': 1, 'b': 'a', 'c': false}, 'one');
        bayes.addExample({'a': 1, 'b': 'b', 'c': false}, 'one');
        bayes.addExample({'a': 4, 'b': 'c', 'c': true}, 'one');
        bayes.addExample({'a': 2, 'b': 'c', 'c': false}, 'two');
        bayes.addExample({'a': 2, 'b': 'd', 'c': false}, 'two');
        bayes.addExample({'a': 2, 'b': 'e'}, 'two');

        bayes.train();

        expect(bayes.classify({'a': 1, 'f': 'e', 'c': true})).toBe('one');
        expect(bayes.classify({'a': 2, 'f': 'r'})).toBe('two');        
    });
});
