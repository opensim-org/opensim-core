
var http = require('http');
var logfmt = require('../logfmt');

var HTTPhandler = function(req, res){
  logfmt.requestLogger()(req,res,function(){
    res.writeHead(200, {'Content-Type': 'text/plain'});
    res.end("Hello, Logfmt\n");
  })
}

http.createServer(HTTPhandler).listen(3000)
console.log("listening on 3000")
