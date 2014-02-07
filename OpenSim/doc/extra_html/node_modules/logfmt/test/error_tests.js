var logfmt = require('../logfmt'),
    assert = require('assert');

var OutStream = require('./outstream');

suite('logfmt.error', function() {
  test('logs an error', function() {
    var err = new Error('testing');
    logfmt.stream = new(require('./outstream'));
    logfmt.error(err);
    var id = logfmt.stream.lines[0].match(/id=(\d+)/)[1];
    assert.equal(logfmt.stream.lines[0], 'error=true id=' + id + ' message=testing\n');
    assert.equal(logfmt.stream.lines[1], 'error=true id=' + id + ' line=0 trace="Error: testing"\n');
  });

  test('sends only a max number of log lines', function() {
    var err = new Error('testing');
    logfmt.stream = new(require('./outstream'));
    logfmt.maxErrorLines = 2;
    logfmt.error(err);
    assert.equal(logfmt.stream.lines.length, 3);
  });
})
