
var context = require('../')

var doc = 
  'hello there, this is a test document...\n' +
  'search this document for matches.\n' +
  'so you can see well the  query matches the document\n' +
  'blah blah blah blah...'

console.log(
  context(doc, ['document', 'query', 'match'])
)
