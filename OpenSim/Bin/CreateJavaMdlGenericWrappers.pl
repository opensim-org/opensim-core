#!/usr/bin/perl -w
#

use Cwd;
$curFolder = cwd;
print "Enter name for Model to use (corresponds to C++ Class to use):";
my $userCurModel = <STDIN>;
chomp($userCurModel);

print "Enter name of the dll/shared_library that represents the model:";
my $userLibName = <STDIN>;
chomp($userLibName);

print "Got userCurModel[".$userCurModel."]";

# This is the place for the standard swig template file
my $swigTemplate ='C:/cygwin/home/Ayman/ReWrap/Trunk/OpenSim/Resources/Swig/genericTemplate.i';

print "Building Swig interface file for model ".$userCurModel."\n\n\n";

my $interfaceFileName = $userCurModel . "JNI.i";
if(open(SWIG_FH, "<$swigTemplate")) {
    if(open(SWIGI_FH, ">$interfaceFileName")) {
	#Loop thru and substitute _MODEL_NAME_HERE_ with $userCurModel
	while(<SWIG_FH>){
	    $nextLine = $_;
	    $nextLine =~ s/_MODEL_NAME_HERE_/$userCurModel/;
	    print SWIGI_FH $nextLine;
	}
	close(SWIGI_FH);
    }
    else {
		die "Cannot open swig interface file";
    }
    close(SWIG_FH);
	
}
else {
	die "Cannot find swig template file";
}

# Now that the file $userCurModel.i is generated, build the dll for the model
# This is done on four steps:
# 1. Generate Swig wrappers
# 2. Update the make files to include the generated JNI file.
# 3. Compile all java files in directory
# 4. Generate jar file

# 1. Generate Swig wrappers
print "Generating Swig wrapper classes for java\n\n";

# Makeup a name for the Package 
$modelPkg = "simtkModel";

`swig -v -c++ -java -package $modelPkg -I'C:/cygwin/home/Ayman/ReWrap/Trunk/' -I'C:/cygwin/home/Ayman/ReWrap/Trunk/OpenSim/Resources/Swig' $interfaceFileName`;

# Now move all java generated files into $modelPkg directory for better directory mgmt.
use File::Copy;
if (mkdir($modelPkg,0777)==0){
	print "Err : $!. Please remove directory $modelPkg manually then retry.\n"; 
	exit(1);
}
@files = <$curFolder/*>; 
for $fil(@files){ 
# Split at the last / and check if it contains .java, if so compile
	@filepath = split(/\\|\//, $fil);
	$curFile = $filepath[$#filepath];
	@nameExtArray = split(/\./, $curFile);
	if ($nameExtArray[$#nameExtArray] eq "java"){
#		print "Moving file ".$curFile."\n";
		move($curFile, $modelPkg."/".$curFile);
	}
}

chdir $modelPkg;

# Create a Loader file to contain the system.loadLibrary and as a main file for the jar
my $LoaderTemplate ='C:/cygwin/home/Ayman/ReWrap/Trunk/OpenSim/Resources/Java/ModelLoaderTemplate.java';

print "Building Java main file for model ".$userCurModel."\n\n\n";

# my $javabuildlog =`"$ENV{JAVA_HOME}"/bin/javac *.java`;	
	
chdir "..";

# The rest of the steps need to be executed after the C++ dll including the wrappers is created

