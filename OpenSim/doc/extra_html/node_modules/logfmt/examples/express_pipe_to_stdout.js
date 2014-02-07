var app    = require('express')();
var assert = require('assert');
var http   = require('http');
var through = require('through');
var logfmt  = require('../logfmt');

app.use(logfmt.bodyParserStream());
app.use(logfmt.requestLogger());

app.post('/logs', function(req, res){
  if(!req.body) return res.send('OK');

  req.body.pipe(through(function(line){
    console.log(JSON.stringify(line));
  }))

  res.send('OK');
})

http.createServer(app).listen(3000);
console.log("Express server listening on port 3000")
