var logfmt = require('../logfmt'),
    assert = require('assert');

var OutStream = require('./outstream');
var mock_sink = new OutStream;

suite('logfmt.log', function() {
  test("logs simple key value pairs", function(){
    var data = {foo: 'bar', a: 14}
    logfmt.log(data, mock_sink);
    assert.equal("foo=bar a=14\n", mock_sink.logline)
  })

  test("logs true and false as strings", function(){
    var data = {foo: true, bar: false}
    logfmt.log(data, mock_sink);
    assert.equal("foo=true bar=false\n", mock_sink.logline)
  })

  test("quotes strings with spaces in them", function(){
    var data = {foo: "hello kitty"}
    logfmt.log(data, mock_sink);
    assert.equal("foo=\"hello kitty\"\n", mock_sink.logline)
  })

  test("quotes strings with equals in them", function(){
    var data = {foo: "hello=kitty"}
    logfmt.log(data, mock_sink);
    assert.equal("foo=\"hello=kitty\"\n", mock_sink.logline)
  })

  test("escapes quotes within strings with spaces in them", function(){
    var data = {foo: 'hello my "friend"'}
    logfmt.log(data, mock_sink);
    assert.equal('foo="hello my \\"friend\\""\n', mock_sink.logline)
    var data = {foo: 'hello my "friend" whom I "love"'}
    logfmt.log(data, mock_sink);
    assert.equal('foo="hello my \\"friend\\" whom I \\"love\\""\n', mock_sink.logline)
  })

  test("undefined is logged as nothing", function(){
    var data = {foo: undefined}
    logfmt.log(data, mock_sink);
    assert.equal("foo=\n", mock_sink.logline)
  })

  test("null is logged as nothing", function(){
    var data = {foo: null}
    logfmt.log(data, mock_sink);
    assert.equal("foo=\n", mock_sink.logline)
  })

  test("setting sink at object level", function(){
    var data = {foo: "hello kitty"}
    var stream = logfmt.stream;
    logfmt.stream = mock_sink;
    logfmt.log(data);
    assert.equal("foo=\"hello kitty\"\n", mock_sink.logline)
    logfmt.stream = stream;
  })
})
