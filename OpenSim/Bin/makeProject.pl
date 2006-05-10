#!/usr/bin/perl -w
#

use Cwd;
$curFolder = cwd;

# Parse curFolder to extract model name
@dirArray = split(/\\|\//, $curFolder);
$subjectName = $dirArray[$#dirArray];
chomp($subjectName);

use locale;

my $CapSubjectName = "\u$subjectName";
my $suCapSubjectName = "su".$CapSubjectName;

chomp($subjectName);
%filemap = (
	"template.sln" => $suCapSubjectName.".sln",
	"template.dsw" => $suCapSubjectName.".dsw",
	"template.h" => $suCapSubjectName.".h",
	"template.cpp" => $suCapSubjectName.".cpp",
	"sdufuncs_darryl.c" => "sdufuncs_darryl.c",
# Project and source files to support running CMC
	"template.trk" => $subjectName.".trk",
	"template_constraints.ctr" => $subjectName."_constraints.ctr",
	"template_CMC.cpp" => $suCapSubjectName."_CMC.cpp",
	"template_CMC.vcproj" => $suCapSubjectName."_CMC.vcproj",
# Project and source files to support forward simulation
	"template_Forward.cpp" => $suCapSubjectName."_Forward.cpp",
	"template_Forward.vcproj" => $suCapSubjectName."_Forward.vcproj",
# Project and source files to support forward simulation with perturbations
	"template_Perturb.cpp" => $suCapSubjectName."_Perturb.cpp",
	"template_Perturb.vcproj" => $suCapSubjectName."_Perturb.vcproj",
	"CMakeLists.txt" => "CMakeLists.txt"
);
my $templateDir = $ENV{NMBL_INSTALL}.'/Template/';

if (mkdir('Results',0777)==0){
	print "Err : $!. Directory could not be created\n"; 
}
print "Created directory Results to save analysis results\n";

if (mkdir('ResultsPerturbed',0777)==0){
	print "Err : $!. Directory could not be created\n"; 
}
print "Created directory ResultsPerturbed to save perturbation analysis results\n";

while ( my ($key, $value) = each(%filemap) ) {
		my $templateFile = $templateDir.$key;
		my $subjectFile = $value;
	if(open(TEMPL_FH, "<$templateFile")) {
		if(open(SUBJECT_FH, ">$subjectFile")) {
		#Loop thru and substitute _template_ with $subjectName
		while(<TEMPL_FH>){
			$nextLine = $_;
			$nextLine =~ s/_template_/$subjectName/g;
			$nextLine =~ s/_Template_/$CapSubjectName/g;
			print SUBJECT_FH $nextLine;
		}
		close(SUBJECT_FH);
		print "Created file $subjectFile\n";
		}
		else {
			die "Cannot open subject file";
		}
		close(SUBJECT_FH);
	}
	else {
		die "Cannot find template file $templateFile";
	}
};
