logfmt = require('../logfmt');

exports.log = function(data, stream) {
  if(stream == undefined) stream = this.stream;

  var line = '';
  Object.keys(data).forEach(function(key){
    var value = data[key];
    var is_null = false;
    if(value == null) {
      is_null = true;
      value = '';
    }
    else value = value.toString();

    var needs_quoting = value.indexOf(' ') > -1 || value.indexOf('=') > -1;
    var needs_escaping = value.indexOf('"') > -1;

    if(needs_escaping) value = value.replace(/"/g, '\\"');
    if(needs_quoting) value = '"' + value + '"';
    if(value === '' && !is_null) value = '""';

    line += key + '=' + value + ' ';
  })

  //trim traling space and print w. newline
  stream.write(line.substring(0,line.length-1) + "\n");
}

exports.time = function(arg1, arg2, arg3) {
  var startTime = (new Date()).getTime();

  var label = arg1;
  var top_data = arg2 || {};
  var callback = arg3;

  if(!arg2){
    label = 'elapsed';
    callback = arg1;
  }
  else if(!arg3){
    if(arg1.substring){
      label = arg1;
    }else{
      label = 'elapsed';
      top_data = arg1;
    }
    callback = arg2;
  }

  var logger = {};
  var self = this;
  logger.log = function(data, stream){
    var now = (new Date()).getTime()
    if(!data) data = {};
    for (key in top_data) {
      data[key] = top_data[key];
    }
    data[label] = (now - startTime).toString() + 'ms' ;
    var stream = stream || self.stream
    self.log(data, stream);
  }

  logger.error = function(err, stream){
    var now = (new Date()).getTime()
    var id = Math.random().toString().slice(2, 12);
    data = { error:true, id:id };
    for (key in top_data) {
      data[key] = top_data[key];
    }
    data[label] = (now - startTime).toString() + 'ms';
    var stream = stream || self.stream;
    self.log(data, stream);
    self.error(err, id);
  }

  if (typeof callback === 'function') {
    callback(logger);
  }

  return logger;
}

exports.namespace = function(object){
  var new_logfmt = new logfmt;
  var old_log = this.log;
  var old_stream = this.stream;
  new_logfmt.log = function(data, stream){
    if(stream == undefined) stream = old_stream;
    data = data || {};
    logdata = {};
    for (key in object) {
      logdata[key] = object[key];
    }
    for (key in data) {
      logdata[key] = data[key];
    }
    old_log(logdata, stream);
  }
  return new_logfmt;
}

exports.error = function(err, id) {
  if (id === undefined) {
    id = Math.random().toString().slice(2, 12);
  }
  this.log({ error:true, id:id, message:err.message });
  var stack = err.stack.split('\n');
  for (var line in stack) {
    if (line >= (this.maxErrorLines || 10)) break;
    this.log({ error:true, id:id, line:line, trace:stack[line] });
  }
}
