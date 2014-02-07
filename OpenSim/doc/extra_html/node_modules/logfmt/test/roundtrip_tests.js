var logfmt = require('../logfmt'),
    assert = require('assert');

var OutStream = require('./outstream');
var mock_sink = new OutStream;

suite('logfmt.parse(logfmt.log)', function(){
  test("key value pairs are restored", function(){
    var data = {foo: 'bar', a: 14}
    logfmt.log(data, mock_sink);
    assert.deepEqual(data, logfmt.parse(mock_sink.logline));
  })

  test("true and false are restored", function(){
    var data = {foo: true, bar: false}
    logfmt.log(data, mock_sink);
    assert.deepEqual(data, logfmt.parse(mock_sink.logline));
  })

  test("quoted strings with spaces are restored", function(){
    var data = {foo: "hello kitty"}
    logfmt.log(data, mock_sink);
    assert.deepEqual(data, logfmt.parse(mock_sink.logline))
  })

  test("quoted strings with equals are restored", function(){
    var data = {foo: "hello=kitty"}
    logfmt.log(data, mock_sink);
    assert.deepEqual(data, logfmt.parse(mock_sink.logline))
  })

  test("escaped strings are restored", function(){
    var data = {foo: 'hello my "friend"'}
    logfmt.log(data, mock_sink);
    assert.deepEqual(data, logfmt.parse(mock_sink.logline))
    data = {foo: 'hello my "friend" whom I "love"'}
    logfmt.log(data, mock_sink);
    assert.deepEqual(data, logfmt.parse(mock_sink.logline))
  })

  test("null comes back as null", function(){
    var data = {foo: null}
    logfmt.log(data, mock_sink);
    assert.deepEqual({foo: null}, logfmt.parse(mock_sink.logline))
  })

  test("empty string comes back as an empty string", function(){
    var data = {foo: ''}
    logfmt.log(data, mock_sink);
    assert.deepEqual({foo: ''}, logfmt.parse(mock_sink.logline))
  })
})
