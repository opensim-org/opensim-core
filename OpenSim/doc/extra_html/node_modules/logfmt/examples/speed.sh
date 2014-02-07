#! /usr/bin/env node

var logfmt = require('../logfmt');

logfmt.old_parse = function(line) {

  var pairs = line.match(/([a-zA-Z0-9\%\_\-\.\^]+)=?(([a-zA-Z0-9\.\-\_\.\/\@]+)|("([^\"]+)"))?/g)
  var attrs = {}

  if(!pairs) { return attrs }

  pairs.forEach(function(pair) {
    parts = pair.split("=")
    key   = parts.shift()
    value = parts.join("=")
    //strip quotes
    if(value[0] == '"'){
      value = value.substring(1, value.length-1)
    }
    //casts
    if(value == '') value = true;
    else if(value == 'true') value = true;
    else if(value == 'false') value = false;
    else if(/^\d+$/.test(value)) value = parseInt(value);
    attrs[key] = value
  })

  return attrs;
}


var args = process.argv.slice(2);
var n = parseInt(args[0]);

console.log('' + n + ' lines');

var time = new Date().getTime();
for(i = 0; i < n; i ++){
  var test_string = "foo=bar a=14 baz=\"hello kitty\" cool%story=bro f %^asdf code=H12";
  logfmt.old_parse(test_string)
}
console.log('parse: ' + (new Date().getTime() - time) + 'ms');

var time = new Date().getTime();
for(i = 0; i < n; i ++){
  var test_string = "foo=bar a=14 baz=\"hello kitty\" cool%story=bro f %^asdf code=H12";
  logfmt.parse(test_string)
}
console.log('parse2: ' + (new Date().getTime() - time) + 'ms');
