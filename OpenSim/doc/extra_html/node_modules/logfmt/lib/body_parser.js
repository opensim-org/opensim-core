
/*
Module dependencies.
*/

/*
JSON:

Parse logplex request bodies, providing the
parsed object as `req.body`.

Options: none

@param content_type {String} use when Content-Type matches this string
@param parser {Function} parsing function takes String body and returns new body
@return {Function}
@api public
*/

exports = module.exports = function(options) {
  var limit;
  if (options == null) options = {};

  return function(req, res, next) {
    if (req._body) return next();
    var is_mime = req.header('content-type') === options.contentType;
    if (!is_mime) return next();
    req._body = true;
    req.body = req.body || {};
    var buf;
    buf = "";
    req.setEncoding("utf8");
    req.on("data", function(chunk) {
      return buf += chunk;
    });
    req.on("end", function() {
      try {
        var lines = []
        buf.trim().split("\n").forEach(function(line){
          lines.push(options.parser(line))
        })
        req.body = lines;
      } catch (err) {
        err.body = buf;
        err.status = 400;
        return next(err);
      }
      return next();
    });
  };
};
