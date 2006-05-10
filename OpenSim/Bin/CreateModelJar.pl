#!/usr/bin/perl -w
#

use Cwd;
$curFolder = cwd;

# Parse curFolder to extract model name
@dirArray = split(/\\|\//, $curFolder);
$curModel = $dirArray[$#dirArray];

$curModelFull = $curModel . "_Model";

$curArch = $ENV{ARCH};
#
# Now handle architecture specific files
#
if (mkdir($curArch,0777)==0){
 	print "Err : $!. Please remove directory $ENV{ARCH} manually then retry.\n"; 
 	exit(1);
}

# find platform specific libraries and copy them to the Arch Directory
%extensions1 = ("VC71", "dll",
         "VC7", "dll",
	 "VC6", "dll");


%extensions2 = ("VC71", "lib",
				"VC7", "lib",
				"VC6", "lib");

use File::Find;
use File::Copy;
#
# ToDo: We need to resolve the issue if old/stale dlls/librraies exist in the path
# We can prompt the user to select current libs but this will require a user interface
# The other option would be to warn the user if multiple ones are found.
#
find (\&copyModelLib, ".");

print "Creating jar file $curModelFull.jar\n\n";

`jar -cf $curModelFull.jar Java $curArch`;

########################################################################
# Function to copy specific platform library files to $ARCH directory
# The directory is assumed to have been created already
# Extensions are extracted from the arrays extensions1, extensions2 based on ARCH
########################################################################

sub copyModelLib {

$curPlatformext1= $extensions1{$curArch};
$curPlatformext2= $extensions2{$curArch};
$ext1 = substr($File::Find::name, length($File::Find::name) - length($curPlatformext1), length($curPlatformext1));
$ext2 = substr($File::Find::name, length($File::Find::name) - length($curPlatformext2), length($curPlatformext2));
@dirArray = split(/\\|\//, $File::Find::name);
$bareFileName = $dirArray[$#dirArray];
#
# We need to makup full path for destFile here since File::Find does chdir internally
#
$destFile = $curFolder."/".$curArch."/".$bareFileName;

# print ("Bare file name =".$bareFileName.", full dest file name =".$destFile."\n");
	if( $ext1 eq $curPlatformext1){
	   print("Copying file ".$File::Find::name." to directory ".$curArch."\n");
	   copy($bareFileName,$destFile) or die "copy failed: $!";

	}
	if( $ext2 eq $curPlatformext2){
	   print("Copying file ".$File::Find::name." to directory ".$curArch."\n");
	   copy($bareFileName,$destFile) or die "copy failed: $!";
	}
}