var app    = require('express')();
var assert = require('assert');
var http   = require('http');
var through = require('through');
var logfmt  = require('../logfmt');

app.use(logfmt.bodyParserStream());

app.post('/logs', function(req, res){
  if(!req.body) return res.send('OK');

  req.body.on('readable', function(){
    var line = req.body.read();
    if(!line) return res.send('OK');
    try {
      console.log(line);
      if(line.at === 'error'){
        assert.equal('H12', line.code)
        assert.equal('50.17.15.69', line.fwd)
      }else{
        assert.equal("bar", line["foo"])
        assert.equal(14, line.a)
        assert.equal("hello kitty", line['baz'])
        assert.equal('bro', line['cool%story'])
        assert.equal(true, line.f)
        assert.equal(true, line['%^asdf'])
      }
    } catch (e) {
      console.log(e)
      res.send('FAIL')
    }
  })
  req.body.resume();
})

http.createServer(app).listen(3000);
console.log("Express server listening on port 3000")
