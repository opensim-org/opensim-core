var split = require('split');
var through = require('through');
var Readable = require('stream').Readable;

exports = module.exports = function(options){
  if(options == null) options = {};
  var mime = options.contentType || "application/logplex-1";

  return function(req, res, next) {
    //setup
    if (req._body) return next();
    var is_mime = req.header('content-type') === mime;
    if (!is_mime) return next();
    req._body = true;

    //define Readable body Stream
    req.body = new Readable({ objectMode: true });
    req.body._read = function(n) {
      req.body._paused = false;
    };

    function parseLine(line) {
      if(line) {
        var parsedLine = options.parser(line);
        if(!req.body._paused) req.body._paused = !req.body.push(parsedLine);
      }
    }
    function end() { req.body.push(null); }
    req.pipe(split()).pipe(through(parseLine, end));

    return next();
  }
}

