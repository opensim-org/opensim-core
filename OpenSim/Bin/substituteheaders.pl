use File::Find;

use File::Copy;

# Set the variable $File::Find::dont_use_nlink if you're using AFS,
# since AFS cheats.

# for the convenience of &wanted calls, including -eval statements:
use vars qw/*name *dir *prune/;
*name   = *File::Find::name;
*dir    = *File::Find::dir;
*prune  = *File::Find::prune;

sub buildHeadersHash;
sub processHeaders;
sub processFile;
sub makeRelative;

my %headersTable ={};

use vars qw(%headersTable);

my $relativepath;
use vars qw($relativepath);

my $relative;
use vars qw($relative);

# Traverse desired filesystems
# Initial directory for traversal and substitution
my $initialDirectory = $ENV{'RDI_DEV'}.'/Source/Native';
# Top include directory as specified by project files in either install or devl environments
my $topIncludeDirectory = $ENV{'RDI_DEV'}.'/Source/Native';
$topIncludeDirectory =~ s/\\/\//g;
use vars qw($topIncludeDirectory);

$initialDirectory =~ s/\\/\//g;
# print $initialDirectory."\n";

File::Find::find({wanted => \&buildHeadersHash}, 'find', $initialDirectory);

my($key, $value); 
while ( ($key, $value) = each(%headersTable) ) {
#    print "key $key Value $value\n";
 }
print "Initial directory is $initialDirectory\n";
File::Find::find({wanted => \&processHeaders}, 'find', $initialDirectory);

exit;


sub buildHeadersHash {
	my $fullname = $File::Find::name;
	my @dirArray = split(/\\|\//, $File::Find::name);
	my $baseFileName = $dirArray[$#dirArray];
	$fullname =~ s/^.\/(.*)/$1/;
    /^.*\.h\z/s and $headersTable{$baseFileName}=$fullname;
    
}

sub processHeaders {
    /^.*\.(h|c)\z/s and processFile;
    /^.*\.(h|cpp|cxx)\z/s and processFile;
}

sub processFile {
	   my $file = $File::Find::name;
	   my $dir = $File::Find::dir;
	   my @dirArray = split(/\\|\//, $file);
	   my $baseFileName = $dirArray[$#dirArray];
	   # print "openning file $baseFileName, current directory $dir\n";
	   open(IN, "< $baseFileName") || die "Cannot open file $baseFileName\n";
	   my $outfilename = $baseFileName."_tmp";
	   open(OUT, "> $outfilename") || die "Cannot open file $outfilename\n"; 
	   while(<IN>) {
	   	  my $nextLine = $_;

	      if (/^#include.*<(.*)>/i) {
	         # file is full path
	         my $sysincludefilename = $1;
	         my $fullsysheaderfilename = $headersTable{$1};
	         $relativepath = makeRelative($fullsysheaderfilename, $dir);
			 if ($relativepath ne ""){	
				$nextLine =~ s/$sysincludefilename/$relativepath/g;
			 }
	         
	      }
	      if (/^#include.*\"(.*)\"/i) {
	         # file is full path
	         my $includefilename = $1;
	         my $fulluserheaderfilename = $headersTable{$1};
	         $relativepath = makeRelative($fulluserheaderfilename, $dir);
			 if ($relativepath ne ""){	
				$nextLine =~ s/$includefilename/$relativepath/g;
			 }
	      }
		  print OUT $nextLine;	      
	   }
	   close(OUT);
	   close(IN);
	   # Keep a backup copy, overwrite existing file with the _tmp file
	   copy($baseFileName, $baseFileName.".bak");
	   move($outfilename, $baseFileName);
}

sub makeRelative {
	my $fullname = @_[0];
	my $currentDir = @_[1];
	# Make all slashes forward to avoid confusion
	$relative = $fullname;
	$relative =~ s/$currentDir/\./;
	
	# Remove leading ./
	$relative =~ s/^.\/(.*)/$1/;
	
	$relative =~ s/$topIncludeDirectory/\./g;
	$relative =~ s/^.\/(.*)/$1/;
#	print "Making relative $fullname and $currentDir, top include dir $topIncludeDirectory result $relative\n";
	
	return $relative;
}