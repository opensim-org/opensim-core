var logfmt = function(stream){
  var parse = require('./lib/logfmt_parser').parse;

  this.parse = parse;
  this.stream = stream || process.stdout;

  var logger = require('./lib/logger');
  this.log = logger.log;
  this.time = logger.time;
  this.namespace = logger.namespace;
  this.error = logger.error;

  this.maxErrorLines = 10;

  //Syncronous Body Parser
  var bodyParser = require('./lib/body_parser')
  this.bodyParser = function(options) {
    if(options == null) options = {};
    var mime = options.contentType || "application/logplex-1"
    return bodyParser({contentType: mime, parser: parse})
  }

  //Stream Body Parser
  var bodyParserStream = require('./lib/body_parser_stream');
  this.bodyParserStream = function(options) {
    if(options == null) options = {};
    var mime = options.contentType || "application/logplex-1"
    return bodyParserStream({contentType: mime, parser: parse})
  }

  //Request Logger
  var requestLogger = require('./lib/request_logger');
  this.requestLogger = function(options, formatter){
    return requestLogger.init(this, options, formatter);
  }
  this.requestLogger.commonFormatter = requestLogger.commonFormatter
}

exports = module.exports = logfmt;
logfmt.call(exports)
