var restify = require('restify');
var logfmt  = require('../logfmt');

var server = restify.createServer({
  name: 'logfmt-test-server'
})

server.use(logfmt.bodyParser());

server.use(function(req,res,next){
  logfmt.time(function(logger){
    var request_data = {
      "method" : req.method,
      "content-type" : req.headers['content-type'],
      "status" : res.statusCode
    }

    next();
    logger.log(request_data);
  })
})

server.post('/logs', function(req, res, next){
  req.body.forEach(function(line){
    console.log(JSON.stringify(line));
  })
  res.send(200, 'OK');
  return next();
})

server.listen(3000);
console.log("server listening on port 3000");
