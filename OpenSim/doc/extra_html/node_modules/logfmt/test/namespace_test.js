var logfmt = require('../logfmt'),
    assert = require('assert');

var OutStream = require('./outstream');

suite('logfmt.namespace', function() {
  test("returns a new logfmt object", function(){
    var logfmt2 = logfmt.namespace();
    var mock_sink = new OutStream;
    var data = {foo: 'bar', a: 14}
    logfmt2.log(data, mock_sink);
    assert.equal("foo=bar a=14\n", mock_sink.logline)
    var recovered = logfmt2.parse(mock_sink.logline)
    assert.deepEqual(data, recovered);
  })

  test("does not modify data passed in", function(){
    var logfmt2 = logfmt.namespace({ns: 'logfmt'});
    var mock_sink = new OutStream;
    var data = {foo: 'bar', a: 14}
    logfmt2.log(data, mock_sink);
    assert.deepEqual(data, {foo: 'bar', a: 14});
  })

  test("includes data passed in on all log lines", function(){
    var logfmt2 = logfmt.namespace({ns: 'logfmt'});
    var mock_sink = new OutStream;
    var data = {foo: 'bar', a: 14}
    logfmt2.log(data, mock_sink);
    assert.equal("ns=logfmt foo=bar a=14\n", mock_sink.logline)
    logfmt2.log({}, mock_sink);
    assert.equal("ns=logfmt\n", mock_sink.logline)
    logfmt2.log(data, mock_sink);
    assert.equal("ns=logfmt foo=bar a=14\n", mock_sink.logline)
  })

  test("can chain namespace calls", function(done){
    var logfmt2 = logfmt.namespace({ns: 'logfmt'})
                        .namespace({thing: 'data'})

    logfmt2.time(function(logger){
      var mock_sink = new OutStream;
      logger.log({}, mock_sink);
      var actual = mock_sink.logline;
      assert(/^ns=logfmt thing=data elapsed=\dms\n$/.test(actual), actual)
      done();
    })
  })

  test("works when a stream is not passed in", function(){
    var logfmt2 = logfmt.namespace();
    var mock_sink = new OutStream;
    var data = '';
    try {
      logfmt2.log({"foo":"bar"});
      data = 'success';
    } catch (err) {
      data = 'failure';
    }
    assert.equal('success', data);
  })
})
