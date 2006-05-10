use strict; 
use File::Find ();

use File::Copy;

# Set the variable $File::Find::dont_use_nlink if you're using AFS,
# since AFS cheats.

# for the convenience of &wanted calls, including -eval statements:
use vars qw/*name *dir *prune/;
*name   = *File::Find::name;
*dir    = *File::Find::dir;
*prune  = *File::Find::prune;

sub processHeaders;
sub processFile;

File::Find::find({wanted => \&processHeaders}, 'find', '.');

exit;


sub processHeaders {
    /^.*\.(h|c)\z/s and processFile($File::Find::name);
    /^.*\.(h|cpp)\z/s and processFile($File::Find::name);
}

sub processFile {
	   my $file = @_[0];
	   open(IN, "< $file") || die "Cannot open file $file\n";;
	   print "openning file @_\n";
	   my $outFile = $file.".nu";
	   open(OUT, "> $outFile") || die "Cannot open file $outFile\n";
	   while(<IN>) {
	      s/^#include <RD(.*)/#include <NMBLTK$1/g;
	      s/^#include <SU(.*)/#include <NMBLTK$1/g;
	      print OUT "$_";
	   }
	   close(IN);
	   close(OUT);
	   my $savefile = $file.".save";
	   move($file, $savefile);
	   move($outFile, $file);
}