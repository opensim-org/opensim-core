#!/usr/bin/perl -w
#

use Cwd;
$curFolder = cwd;

# Parse curFolder to extract model name
@dirArray = split(/\\|\//, $curFolder);
$curModel = $dirArray[$#dirArray];

# This is the place for the standard swig template file
my $swigTemplate =$ENV{RDI_DEV}.'/Source/SU/Resources/Swig/template.i';

print "Building Swig interface file for model ".$curModel."\n\n\n";

my $interfaceFileName = $curModel . "JNI.i";
if(open(SWIG_FH, "<$swigTemplate")) {
    if(open(SWIGI_FH, ">$interfaceFileName")) {
	#Loop thru and substitute _MODEL_NAME_HERE_ with $curModel
	while(<SWIG_FH>){
	    $nextLine = $_;
	    $nextLine =~ s/_MODEL_NAME_HERE_/$curModel/;
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

# Now that the file $curModelJNI.i is generated, build the dll for the model
# This is done on four steps:
# 1. Generate Swig wrappers
# 2. Update the make files to include the generated JNI file.
# 3. Compile all java files in directory
# 4. Generate jar file

# 1. Generate Swig wrappers
print "Generating Swig wrapper classes for java\n\n";

# Makeup a name for the Package 
$modelPkg = "simtk".$curModel."Pkg";

`swig -v -c++ -java -package $modelPkg -I"$ENV{RDI_DEV}"/IncludeDev -I"$ENV{RDI_DEV}"/Source/SU/Resources/Swig $interfaceFileName`;

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
my $LoaderTemplate =$ENV{RDI_DEV}.'/Source/SU/Resources/Java/ModelLoaderTemplate.java';

print "Building Java main file for model ".$curModel."\n\n\n";

my $ModelLoaderFileName = $curModel . "Loader.java";
if(open(LOADER_FH, "<$LoaderTemplate")) {
    if(open(MLOADER_FH, ">$ModelLoaderFileName")) {
	#Loop thru and substitute _MODEL_NAME_HERE_ with $curModel
	while(<LOADER_FH>){
	    $nextLine = $_;
	    $nextLine =~ s/_MODEL_NAME_HERE_/$curModel/g;
	    $nextLine =~ s/_PACKAGE_NAME_HERE_/$modelPkg/g;
	    print MLOADER_FH $nextLine;
	}
	close(MLOADER_FH);
    }
    else {
		die "Cannot create model loader file";
    }
    close(LOADER_FH);
	
}
else {
	die "Cannot find model loader template file";
}



my $javabuildlog =`"$ENV{JAVA_HOME}"/bin/javac *.java`;	
	
chdir "..";

# The rest of the steps need to be executed after the C++ dll including the wrappers is created

