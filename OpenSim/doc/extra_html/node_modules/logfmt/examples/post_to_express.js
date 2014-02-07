var http   = require('http');

var options = {
  hostname: 'localhost',
  port: 3000,
  path: '/logs',
  method: 'POST',
  headers: {
    'Content-Type':'application/logplex-1'
  }
};

var req = http.request(options, function(res) {
  console.log('STATUS: ' + res.statusCode);
  console.log('HEADERS: ' + JSON.stringify(res.headers));
  res.setEncoding('utf8');
  res.pipe(process.stdout)
  res.on('end', function() { process.exit(0) });
});

req.on('error', function(e) {
  console.log('problem with request: ' + e.message);
});

// write data to request body
req.write('foo=bar a=14 baz="hello kitty" cool%story=bro f %^asdf\n');
req.write('at=error code=H12 desc="Request timeout" method=GET path=/users/user38948@heroku.com/usage/2013-05-01T00:00:00Z/2013-05-01T00:00:00Z?exclude=platform%3Adyno%3Aphysical host=vault-usage-read.herokuapp.com fwd="50.17.15.69" dyno=web.21 connect=17ms service=30000ms status=503 bytes=0');
req.end();
