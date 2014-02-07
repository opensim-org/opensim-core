#! /usr/bin/env node

logfmt = require('./lib/logfmt_parser')
logfmt.debug = true;
var test_string = "foo=bar a=14 baz=\"hello kitty\" cool%story=bro f %^asdf ";
test_string += "code=H12 path=/hello/user@foo.com/close";
console.log(logfmt.parse(test_string))
