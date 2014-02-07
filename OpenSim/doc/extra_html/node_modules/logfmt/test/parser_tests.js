var logfmt = require('../logfmt'),
    assert = require('assert');

suite('logfmt.parse', function() {
  test("simple flag parses", function(){
    assert.deepEqual({'hello':true}, logfmt.parse('hello'));
  })

  test("simple key/value parses", function(){
    assert.deepEqual({'hello':'kitty'}, logfmt.parse('hello=kitty'));
  })

  test("simple boolean parses", function(){
    assert.deepEqual({'foo':true, 'bar':false},
      logfmt.parse('foo=true bar=false'));
  })

  test('big numbers dont lose precision', function(){
    var parsed = logfmt.parse("thing=90071992547409934");
    assert.equal(parsed.thing.toString(), '90071992547409934');
  })

  test("number parse to strings", function(){
    assert.deepEqual({'foo':'123', 'bar':'456.789'},
      logfmt.parse('foo=123 bar=456.789'));
  })

  test("string with escapes", function(){
    assert.deepEqual({'hello':"\'kitty\'"},
      logfmt.parse('hello="\'kitty\'"'));
    assert.deepEqual({'hello':"\'kitty\'"},
      logfmt.parse('hello=\'kitty\''));
  })

  test("string with equals", function(){
    assert.deepEqual({foo:"hello=kitty"}, logfmt.parse('foo="hello=kitty"'));
  })

  test("readme string parses", function(){
    var test_string = "foo=bar a=14 baz=\"hello kitty\" "
    test_string += "cool%story=bro f %^asdf ";
    test_string += "code=H12 path=/hello/user@foo.com/close";
    var result = logfmt.parse(test_string)
    assert.equal( "H12", result["code"])
    assert.equal( "bar", result["foo"])
    assert.equal(14, result.a)
    assert.equal("hello kitty", result['baz'])
    assert.equal('bro', result['cool%story'])
    assert.equal(true, result.f)
    assert.equal(true, result['%^asdf'])
    assert.equal('/hello/user@foo.com/close', result['path'])
  })
})
