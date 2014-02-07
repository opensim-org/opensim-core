var logfmt = require('../logfmt');

logfmt.time('thing', function(logger){
  logger.log({foo: 'bar'})
  var this_thing = function(){
    logger.log({at: "thing"})
    logfmt.log({at: "logfmt log"})
    logfmt.time(function(inner_logger){
      var inner_thing = function(){
        inner_logger.log({at: 'inner thing'});
      }
      setTimeout(inner_thing, 100);
    })
  }
  setTimeout(this_thing, 300);
})
