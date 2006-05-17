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

# This is the place for the standard swig template file
my $swigTemplate ='C:/cygwin/home/Ayman/ReWrap/Trunk/OpenSim/Resources/Swig/specificTemplate.i';

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

# Now that the file $userCurModelJNI.i is generated, build the dll for the model
# This is done on four steps:
# 1. Generate Swig wrappers
# 2. Update the make files to include the generated JNI file.
# 3. Compile all java files in directory
# 4. Generate jar file

# 1. Generate Swig wrappers
print "Generating Swig wrapper classes for java\n\n";

# Makeup a name for the Package 
$modelPkg = "simtk".$userCurModel."Pkg";

`swig -v -c++ -java -package $modelPkg -I"C:/cygwin/home/Ayman/ReWrap/Trunk/" -I"C:/cygwin/home/Ayman/ReWrap/Trunk/OpenSim/Resources/Swig" $interfaceFileName`;

# Now move all java generated files into $modelPkg directory for better directory mgmt.
use File::Copy;
if (mkdir($modelPkg,0777)==0){
	print "Err : $!. Please remove directory $modelPkg manually then retry.\n"; 
	exit(1);
}
# The following files need to be execluded as they are part of the platform (generic files)
@simtkFiles = ("Actuator.java",
"ActuatorSet.java",
"ArrayBool.java",
"ArrayDouble.java",
"ArrayInt.java",
"ArrayPtrsObj.java",
"ArrayStr.java",
"Body.java",
"ContactForceSet.java",
"Material.java",
"MaterialManager.java",
"Model.java",
"rdObject.java",
"Property.java",
"PropertySet.java",
"SetActuators.java",
"Transform.java",
"VisibleObject.java",
"VisibleProperties.java",
"SWIGTYPE_p_DOMElement.java",
"SWIGTYPE_p_a_3__double.java",
"SWIGTYPE_p_bool.java",
"SWIGTYPE_p_double.java",
"SWIGTYPE_p_float.java",
"SWIGTYPE_p_int.java",
"SWIGTYPE_p_p_Object.java",
"SWIGTYPE_p_Analysis.java",
"SWIGTYPE_p_ArrayPtrsTActuator_t.java",
"SWIGTYPE_p_ArrayPtrsTProperty_t.java",
"SWIGTYPE_p_ContactForce.java",
"SWIGTYPE_p_DerivCallback.java",
"SWIGTYPE_p_IntegCallbackSet.java",
"SWIGTYPE_p_MaterialManager.java",
"SWIGTYPE_p_PropertySet.java",
"SWIGTYPE_p_Storage.java",
"SWIGTYPE_p_XMLDocument.java",
"SWIGTYPE_p_std__string.java",
"SWIGTYPE_p_void.java") ;
$ending_value = scalar(@simtkFiles) ;


@files = <$curFolder/*>; 

for $fil(@files){ 
# Split at the last / and check if it contains .java, if so compile
	@filepath = split(/\\|\//, $fil);
	$curFile = $filepath[$#filepath];
	@nameExtArray = split(/\./, $curFile);
	if ($nameExtArray[$#nameExtArray] eq "java"){
#		print "Moving file ".$curFile."\n";
		my $found = 0;
		for($counter=0 ; $counter < $ending_value && $found==0; $counter++)
		{
		   $found = ($simtkFiles[$counter] eq $curFile)?1:0;
		}
		if ($found==0){
			$copyFile = $modelPkg."/".$curFile;
			if(open(CUR_FH, "<$curFile")) {
				if (open(SUBST_CUR_FH, ">$copyFile")){
					while(<CUR_FH>){
					#	Here both files are open. Copy lines and insert the import opensimModel.*; line
						$nextLine = $_;
						$nextLine =~ s/^package *([^ ]*)/package $1\nimport opensimModel.*;/g;
						print SUBST_CUR_FH $nextLine;
					}
				}
				else {
					die("Cant open copy file");
				}
				close(SUBST_CUR_FH);
			}
			else {
				die ("Cannot open file".$curFile);
			}
			close(CUR_FH);
		}
	}
}

chdir $modelPkg;

# Create a Loader file to contain the system.loadLibrary and as a main file for the jar
my $LoaderTemplate ='C:/cygwin/home/Ayman/ReWrap/Trunk/OpenSim/Resources/Java/ModelLoaderTemplate.java';

print "Building Java main file for model ".$userCurModel."\n\n\n";

my $ModelLoaderFileName = $userCurModel . "Loader.java";
if(open(LOADER_FH, "<$LoaderTemplate")) {
    if(open(MLOADER_FH, ">$ModelLoaderFileName")) {
	#Loop thru and substitute _MODEL_NAME_HERE_ with $userCurModel
	while(<LOADER_FH>){
	    $nextLine = $_;
	    $nextLine =~ s/_MODEL_NAME_HERE_/$userCurModel/g;
	    $nextLine =~ s/_PACKAGE_NAME_HERE_/$modelPkg/g;
	    $nextLine =~ s/_MODEL_LIBRARY_NAME_HERE_/$userLibName/g;
	    $nextLine =~ s/^package *([^ ]*)/package $1\nimport opensimModel.*;/g;
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


# This is just a sanity check but it's not needed as java source files can be used already
# my $javabuildlog =`"$ENV{JAVA_HOME}"/bin/javac *.java`;	
	
chdir "..";

# The rest of the steps need to be executed after the C++ dll including the wrappers is created

