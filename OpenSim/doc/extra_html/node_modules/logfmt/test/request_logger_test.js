var logfmt = require('../logfmt'),
    assert = require('assert');

var OutStream = require('./outstream');

suite('logfmt.requestLogger', function(){
  setup(function(){
    logfmt.stream = new OutStream;
  })

  test("timing logs on res.end()", function(done){
    var mockReq = {method: 'GET'}
    var mockRes = {statusCode: 200}
    mockRes.end = function(data, encoding){}
    var next = function(){
      assert.equal('', logfmt.stream.logline);
    };

    var logger = logfmt.requestLogger(function(req,res){
      return {
        method: req.method,
        "status": res.statusCode
      }
    });
    logger(mockReq, mockRes, next)
    mockRes.end()
    var expectation = /method=GET status=200 elapsed=\dms\n/
    var actual = logfmt.stream.logline;
    assert(expectation.test(actual), actual);
    done();
  })

  test("immediate option logs before next()", function(done){
    var mockReq = {method: 'GET'}
    var mockRes = {statusCode: 200}
    var next = function(){
      assert.equal('method=GET status=200\n', logfmt.stream.logline);
    };

    var logger = logfmt.requestLogger({immediate: true}, function(req,res){
      return {
        method: req.method,
        "status": res.statusCode
      }
    });
    logger(mockReq, mockRes, next)
    done()
  })

  test("can just send options", function(done){
    var mockReq = {method: 'GET'}
    mockReq.header = function(){
      return 'foo';
    }
    var mockRes = {statusCode: 200}
    mockRes.get = function(){
      return 'foo';
    }
    var next = function(){
      var actual = logfmt.parse(logfmt.stream.logline);
      assert.equal('foo', actual.ip);
      assert.equal('foo', actual.content_type);
      assert.equal('foo', actual.content_length);
    };

    var logger = logfmt.requestLogger({immediate: true})
    logger(mockReq, mockRes, next)
    done()
  })

  test("elapsed option renames elapsed key", function(done){
    var mockReq = {method: 'GET'}
    var mockRes = {statusCode: 200}
    mockRes.end = function(data, encoding){}
    var next = function(){
      assert.equal('', logfmt.stream.logline);
    };

    var logger = logfmt.requestLogger({elapsed: 'time'}, function(req,res){
      return {
        method: req.method,
        "status": res.statusCode
      }
    });
    logger(mockReq, mockRes, next)
    mockRes.end()
    var expectation = /method=GET status=200 time=\dms\n/
    var actual = logfmt.stream.logline;
    assert(expectation.test(actual), actual);
    done()
  })

  test("empty defaults to commonLogger", function(done){
    var mockReq = {method: 'GET'}
    mockReq.path = '/bar'
    mockReq.ip = '1.0.0.1'
    mockReq.header = function(h){
      return 'foo'
    }
    var mockRes = {statusCode: 200}
    var headers = {
      "content-type": 'foo/foo',
      "content-length": 123
    }
    mockRes.get = function(h){
      return headers[h];
    }
    mockRes.end = function(data, encoding){}
    var next = function(){
      assert.equal('', logfmt.stream.logline);
    };
    var logger = logfmt.requestLogger();
    logger(mockReq, mockRes, next)
    mockRes.end()
    var actual = logfmt.parse(logfmt.stream.logline);
    assert.equal(actual.path, '/bar');
    assert.equal(actual.ip, '1.0.0.1');
    assert.equal(actual.content_type, 'foo/foo');
    assert.equal(actual.content_length, '123');
    assert(/\dms/.test(actual.elapsed), 'includes elapsed')
    assert(actual.time)
    done();
  })

  test("commonFormatter uses correct path", function(){
    var mockReq = {method: 'GET'}
    mockReq.path = function(){ return '/bar' }
    mockReq.ip = '1.0.0.1'
    mockReq.header = function(h){ return 'foo' }
    var mockRes = {statusCode: 200}
    mockRes.get = function(){ return 'foo' }
    var actual = logfmt.requestLogger.commonFormatter(mockReq, mockRes);
    assert.equal('/bar', actual.path);
  })

  test("commonFormatter uses correct path w. vanilla http", function(){
    var mockReq = {method: 'GET'}
    mockReq.url = '/bar'
    mockReq.ip = '1.0.0.1'
    var mockRes = {statusCode: 200}
    var actual = logfmt.requestLogger.commonFormatter(mockReq, mockRes);
    assert.equal('/bar', actual.path);
  })

  test("commonFormatter uses correct ip", function(){
    var mockReq = {method: 'GET'}
    mockReq.path = '/bar'
    var headers = {
      "x-forwarded-for": '10.0.1.1'
    }
    mockReq.header = function(h){
      return headers[h] || 'foo';
    }
    var mockRes = {statusCode: 200}
    mockRes.get = function(){
      return 'foo';
    }
    var actual = logfmt.requestLogger.commonFormatter(mockReq, mockRes);
    assert.equal('10.0.1.1', actual.ip);
  })

  test("requestLogger works with namespace", function(done){
    var mockReq    = {method: 'GET'}
    mockReq.header = function(){ return 'foo'; }
    var mockRes    = {statusCode: 200}
    mockRes.get    = function(){ return 'foo'; }
    var next = function(){
      var actual = logfmt.parse(logfmt.stream.logline);
      assert.equal('namespacetest', actual.ns);
    };

    var logger = logfmt.namespace({ns:'namespacetest'}).requestLogger({immediate: true})
    logger(mockReq, mockRes, next)
    done()
  })

})
