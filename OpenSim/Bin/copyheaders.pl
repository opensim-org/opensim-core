use File::Find ();

use File::Copy;
use File::Copy cp;

# Set the variable $File::Find::dont_use_nlink if you're using AFS,
# since AFS cheats.

# for the convenience of &wanted calls, including -eval statements:
use vars qw/*name *dir *prune/;
*name   = *File::Find::name;
*dir    = *File::Find::dir;
*prune  = *File::Find::prune;

sub buildHeadersHash;
sub findRelativePath;

$destDir = $ENV{'RDI_DEV'}.'/Include/';
$srcDir = $ENV{'RDI_DEV'}.'/Source';

@donotCopy = ('CVS', 'CVSROOT', 'Repository', 'Entries', 'Root', 'makefile', 'VisualStudio', 'Bin', 'Release', 'Debug');

if (!-d $destDir){
	mkdir($destDir);
}
# Traverse desired filesystems, create parallel directories and copy header files
File::Find::find({wanted => \&copyheader}, 'find', $srcDir);

# Remove empty directory in destination
chdir ($destDir);
use File::Find;
finddepth(sub{rmdir},$destDir);

exit;


sub copyheader {
	my @dirArray = split(/\\|\//, $File::Find::name);
	$bareFileName = $dirArray[$#dirArray];
	my $relativePath = findRelativePath($File::Find::name, $srcDir);
	# Select only header files
    if ($bareFileName =~ /^.*\.h$/){
		cp($bareFileName, $destDir.$relativePath);
	}
	else {
		my $is_reserved = false;
		for my $nextReservedName (@donotCopy) {
		if ($nextReservedName eq $bareFileName){
			$is_reserved = true;
			}
		}
		if (-d $bareFileName && $is_reserved eq false){
			mkdir $destDir.$relativePath;
#			print "Created directory $destDir"."$relativePath\n";
		}
	}
}

sub findRelativePath {
	my $arg1 = @_[0];
	my $arg2 = @_[1];
	# Make all slashes forward to avoid confusion
	$arg1 =~ s/\\/\//g;
	$arg2 =~ s/\\/\//g;
	my $relative = $arg1;
	$relative =~ s/$arg2/\./;
	
	# Remove leading ./
	$relative =~ s/^.\/(.*)/$1/;
	
	return $relative;
}