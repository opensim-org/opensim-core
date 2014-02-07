var http = require('http');
var express = require("express");
var logfmt = require("logfmt");
var fs = require('fs');
var si = require('search-index');
var xml2js = require('xml2js');

var app = express();

app.use(logfmt.requestLogger());
app.use(express.bodyParser());

app.configure(function() {
  app.use(express.static(__dirname + '/'));
  app.set('port', Number(process.env.PORT || 8080));
});

function getQuery(req) {
  //default values
  var offsetDefault = 0,
      pagesizeDefault = 10,
      q = {};
  q['query'] = "*";
  if (req.query['q']) {
    q['query'] = req.query['q'].toLowerCase().split(' ');
  }
  if (req.query['searchFields']) {
    q['searchFields'] = req.query.searchFields;
  }
  if (req.query['offset']) {
    q['offset'] = req.query['offset'];
  } else {
    q['offset'] = offsetDefault;
  }
  if (req.query['pagesize']) {
    q['pageSize'] = req.query['pagesize'];
  } else {
    q['pageSize'] = pagesizeDefault;
  }
  if (req.query['facets']) {
    q['facets'] = req.query['facets'].toLowerCase().split(',');
  }
  if (req.query['weight']) {
    q['weight'] = req.query.weight;
  }
  if (req.query['teaser']) {
    q['teaser'] = req.query.teaser;
  }
  if (req.query['filter']) {
    q['filter'] = req.query.filter;
  }
  console.log(q);
  return q;
}


app.get("/query", function (req, res) {
  var q = getQuery(req);
  res.set("Access-Control-Allow-Origin", "*");
  si.search(q, function(msg) {
    res.send(msg);
  });
});

app.get("/ping", function (req, res) {
    res.send("OK");
});

var parser = new xml2js.Parser({ mergeAttrs : true });

app.post('/indexer', function(req, res) {
  console.log('[indexing batch] ' + req.files.document.name);
  var filters = [];
  if (req.body.filterOn) {
    filters = req.body.filterOn.split(',');
  }
  fs.readFile(req.files.document.path, {'encoding': 'utf8'}, function(err, brrr) {
    if (err) console.log(err);
    if(err) return res.send(500, 'Error reading document');

  parser.parseString(brrr, function (err, result) {

    var parsed = result;
    for(i=0; i < parsed.add.doc.length; i++)
    {
      var txt_n = "";
      var txt_cap = "";
      var href = "";
      var type = "";
      for(j=0; j < parsed.add.doc[i].field.length; j++)
      {
        if (parsed.add.doc[i].field[j].name[0] == "text")
          txt_n += parsed.add.doc[i].field[j]._;
        if (parsed.add.doc[i].field[j].name[0] == "name")
          txt_cap += parsed.add.doc[i].field[j]._;
        if (parsed.add.doc[i].field[j].name[0] == "url")
          href += parsed.add.doc[i].field[j]._;
        if (parsed.add.doc[i].field[j].name[0] == "type")
          type += parsed.add.doc[i].field[j]._;
      }
      var doc = {};
      doc.title = txt_cap;
      doc.type = type;
      doc.body = txt_n;
      var batch = {};
      batch[href] = doc;
      si.index(JSON.stringify(batch), href, filters, function(msg) {
      });
	
    }
	res.end("OK");
  });
  
  });
});

http.createServer(app).listen(app.get('port'), function(){
	console.log( 'Listening (' + app.get('env') + ') on port ' + app.get('port'));
});
