#!/usr/bin/perl -w
#

use Cwd;
$curFolder = cwd;

# Parse curFolder to extract model name
@dirArray = split(/\\|\//, $curFolder);
$subjectName = $dirArray[$#dirArray];
chomp($subjectName);

use locale;

if ($ARGV[0]) {
	$subjectName = $ARGV[0];
	print "Using subject name ".$subjectName."\n";
} else {
	print "Use subject name ".$subjectName." (y/n)?";
	$response = <STDIN>;
	if ($response =~ m/^[N|n]$/) {
		print "Enter subject name :";
		$subjectName = readline STDIN;
		chomp($subjectName);	
	}
}

chomp($subjectName);
%filemap = (
	"template.sln" => $subjectName.".sln",
	"template.vcproj" => $subjectName.".vcproj"
);
my $templateDir = $ENV{OPENSIM_TEMPLATE_DIR};

if (!($templateDir)){
	die "\nTemplate directory for creating projects is not set.\nPlease set the environment variable OPENSIM_TEMPLATE_DIR\nto point to the directory name ending in 'Template' in your installation\n";
}
my $lastchar = substr($templateDir, -1);

if (!($lastchar eq '/')) {$templateDir .= "/";}
	
while ( my ($key, $value) = each(%filemap) ) {
		my $templateFile = $templateDir.$key;
		my $subjectFile = $value;
	if(open(TEMPL_FH, "<$templateFile")) {
		if(open(SUBJECT_FH, ">$subjectFile")) {
		#Loop thru and substitute _template_ with $subjectName
		while(<TEMPL_FH>){
			$nextLine = $_;
			$nextLine =~ s/_template_/$subjectName/g;
			$nextLine =~ s/_Template_/$subjectName/g;
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
