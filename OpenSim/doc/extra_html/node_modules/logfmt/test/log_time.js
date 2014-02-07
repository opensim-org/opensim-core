var logfmt = new require('../logfmt'),
    assert = require('assert');

var OutStream = require('./outstream');

suite('logfmt.time', function() {
  setup(function(){
    logfmt.stream = new OutStream;
  })

  test("logs the time", function(done){
    logfmt.time(function(logger){
      logger.log();
      var actual = logfmt.stream.logline;
      assert(/^elapsed=\dms\n$/.test(actual), actual)
      done();
    })
  })

  test("logs the time with your label", function(done){
    logfmt.time('time', function(logger){
      logger.log();
      var actual = logfmt.stream.logline;
      assert(/^time=\dms\n$/.test(actual), actual)
      done();
    })
  })

  test("logs the time with your label and persistent data", function(done){
    logfmt.time('time', {foo: 'bar'}, function(logger){
      logger.log();
      var actual = logfmt.stream.logline;
      assert(/^foo=bar time=\dms\n$/.test(actual), actual)
      done();
    })
  })

  test("logs the time with persistent data", function(done){
    logfmt.time({foo: 'bar'}, function(logger){
      logger.log();
      var actual = logfmt.stream.logline;
      assert(/^foo=bar elapsed=\d+ms\n$/.test(actual), actual)
      logger.log({moar: 'data'});
      var actual = logfmt.stream.logline;
      assert(/^moar=data foo=bar elapsed=\d+ms\n$/.test(actual), actual)
      done();
    })
  })

  //now we're using setTimeout to ensure the elapsed
  //time reflects a known delay
  test("accurancy in milliseconds", function(done){
    logfmt.time(function(logger){
      var wrapped = function() {
        logger.log();
        var actual = logfmt.stream.logline;
        assert(/^elapsed=2\dms\n$/.test(actual), actual)
        done();
      }
      setTimeout(wrapped, 20);
    })
  })

  test("logs the time with your label and data", function(done){
    logfmt.time('time', function(logger){
      logger.log({foo: 'bar'});
      var actual = logfmt.stream.logline;
      assert(/^foo=bar time=\d+ms\n$/.test(actual), actual)
      done();
    })
  })

  test("supports log(data, stream) interface", function(done){
    var mock_sink = new OutStream;
    logfmt.time(function(logger){
      logger.log({foo: 'bar'}, mock_sink);
      var actual = mock_sink.logline;
      assert(/^foo=bar elapsed=\d+ms\n$/.test(actual), actual)
      done();
    })
  })

  test("calls the callback if provided", function(){
    var test;
    logfmt.time(function(logger){
      test = true;
    });
    assert(test);
  })

  test("does not call the callback if not provided", function(){
    assert.doesNotThrow(function(){
      logfmt.time();
    });
  })

  test("returns a logger", function(){
    var logger1, logger2;
    logger1 = logfmt.time(function(logger){
      logger2 = logger;
    });
    assert.equal(logger1, logger2);
  })

  // tests you can pass the logger into a closure
  // and call `log` multiple times.
  // uses setTimeout to ensure the timing happens in 20ms
  test("can log twice", function(done){
    var mock_sink = new OutStream;
    logfmt.time(function(logger){
      logger.log({foo: 'bar'}, mock_sink);
      var actual = mock_sink.logline;
      assert(/^foo=bar elapsed=\d+ms\n$/.test(actual), actual)
      var wrapped = function() {
        logger.log({bar: 'foo'}, mock_sink);
        var actual = mock_sink.logline;
        assert(/^bar=foo elapsed=2\d+ms\n$/.test(actual), actual)
        done();
      }
      setTimeout(wrapped, 20);
    })
  })
})
