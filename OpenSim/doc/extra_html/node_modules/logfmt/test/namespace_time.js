var logfmt = require('../logfmt'),
    assert = require('assert');

var OutStream = require('./outstream');

suite('logfmt.namespace.time', function() {
  test("works with explicit logging location", function(done){
    var logfmt2 = logfmt.namespace({ns: 'logfmt'})
    logfmt2.time(function(logger){
      var mock_sink = new OutStream;
      logger.log({}, mock_sink);
      var actual = mock_sink.logline;
      assert(/^ns=logfmt elapsed=\dms\n$/.test(actual), actual)
      done();
    })
  })

  test("works with logfmt.time and implicit log location", function(done){
    var logfmt2 = logfmt.namespace({ns: 'logfmt'})
    logfmt2.stream = new OutStream;

    logfmt2.time('time', function(logger){
      logger.log({foo: 'bar'});
      var actual = logfmt2.stream.logline;
      assert(/^ns=logfmt foo=bar time=\dms\n$/.test(actual), actual)
      done();
    })
  })

  test("works with persistent data", function(done){
    var logfmt2 = logfmt.namespace({ns: 'logfmt'})
    logfmt2.stream = new OutStream;

    logfmt2.time({foo: 'bar'}, function(logger){
      logger.log();
      var actual = logfmt2.stream.logline;
      assert(/^ns=logfmt foo=bar elapsed=\dms\n$/.test(actual), actual)
      logger.log({moar: 'data'});
      var actual = logfmt2.stream.logline;
      assert(/^ns=logfmt moar=data foo=bar elapsed=\dms\n$/.test(actual),
              actual)
      done();
    })
  })
})
