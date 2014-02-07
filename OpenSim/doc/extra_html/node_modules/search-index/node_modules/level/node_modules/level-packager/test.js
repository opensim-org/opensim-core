const fs    = require('fs')
    , path  = require('path')

var location = path.join(__dirname, 'level-test-' + process.pid + '.db')

module.exports = function (test, level, options) {
  options = options || {}

  test('test db open and use, level(location, cb)', function (t) {
    level(location, function (err, db) {
      t.notOk(err, 'no error')
      db.put('test1', 'success', function (err) {
        t.notOk(err, 'no error')
        db.close(t.end.bind(t))
      })
    })
  })

  test('test db open and use, level(location, options, cb)', function (t) {
    level(location, { createIfMissing: false, errorIfExists: false }, function (err, db) {
      t.notOk(err, 'no error')
      db.put('test2', 'success', function (err) {
        t.notOk(err, 'no error')
        db.close(t.end.bind(t))
      })
    })
  })

  if (!options.skipErrorIfExistsTest) {
    // should use existing options object
    test('test db open and use, level(location, options, cb) force error', function (t) {
      level(location, { errorIfExists: true }, function (err, db) {
        t.ok(err, 'got error opening existing db')
        t.notOk(db, 'no db')
        t.end()
      })
    })
  }

  test('test db open and use, db=level(location)', function (t) {
    var db = level(location)
    db.put('test3', 'success', function (err) {
      t.notOk(err, 'no error')
      db.close(t.end.bind(t))
    })
  })

  test('test db values', function (t) {
    var c  = 0
      , db = level(location)
      , setup = options.nonPersistent
          ? function (callback) {
              db.batch([
                  { type: 'put', key: 'test1', value: 'success' }
                , { type: 'put', key: 'test2', value: 'success' }
                , { type: 'put', key: 'test3', value: 'success' }
              ], callback)
            }
          : function (callback) { callback() }

    function read (err, value) {
      t.notOk(err, 'no error')
      t.equal(value, 'success')
      if (++c == 3)
        db.close(t.end.bind(t))
    }

    setup(function (err) {
      t.notOk(err, 'no error')
      db.get('test1', read)
      db.get('test2', read)
      db.get('test3', read)
    })
  })

  if (!options.skipRepairTest) {
    test('test repair', function (t) {
      t.plan(1)
      level.repair(location, function (err) {
        t.notOk(err, 'no error')
      })
    })
  }

  if (!options.skipDestroyTest) {
    test('test destroy', function (t) {
      t.plan(4)
      t.ok(fs.statSync(location).isDirectory(), 'sanity check, directory exists')
      t.ok(fs.existsSync(path.join(location, 'LOG')), 'sanity check, log exists')
      level.destroy(location, function (err) {
        t.notOk(err, 'no error')
        t.notOk(fs.existsSync(path.join(location, 'LOG')), 'db gone (mostly)')
      })
    })
  }
}
