var logfmt = require('../logfmt');


logfmt.log({hello: 'stdout'});
//=> hello=stdout

errorLogger = new logfmt;
errorLogger.stream = process.stderr

errorLogger.log({hello: 'stderr'});
//=> hello=stderr
