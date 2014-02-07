exports = module.exports = function OutStream(){
  this.lines = [];
  this.logline = '';
  this.write = function(string) {
    this.logline = string;
    this.lines.push(string);
  }
}
