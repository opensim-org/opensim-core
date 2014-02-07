var app    = require('express')();
var assert = require('assert');
var http   = require('http');

var logfmt   = require('../logfmt');

app.use(logfmt.bodyParser());

app.use(logfmt.requestLogger(function(req,res){
  return {
    "method" : req.method,
    "content-type" : req.headers['content-type'],
    "status" : res.statusCode
  }
}))

app.post('/logs', function(req, res){
  console.log('HEADERS: ' + JSON.stringify(req.headers));
  console.log('BODY: ' + JSON.stringify(req.body));
  try {
    var result = req.body[0];
    assert.equal( "bar", result["foo"])
    assert.equal(14, result.a)
    assert.equal("hello kitty", result['baz'])
    assert.equal('bro', result['cool%story'])
    assert.equal(true, result.f)
    assert.equal(true, result['%^asdf'])
    result = req.body[1];
    assert.equal('H12', result.code)
    assert.equal('50.17.15.69', result.fwd)
    res.send('OK')
  } catch (e) {
    res.send('FAIL')
  }
})

http.createServer(app).listen(3000);
console.log("Express server listening on port 3000")

