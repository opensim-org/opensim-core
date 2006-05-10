#!/usr/bin/perl -w
#
print "Enter joint file name for subject :";
my $filename = <STDIN>;
chomp($filename);
my $bareFileName=$filename;
if ($filename =~ /(.*).jnt/)
{
	$bareFileName=$1;
}
else {
	$filename =$filename.".jnt";
}
my $outFileName=$bareFileName."_geom.h";

open TXT, "< $filename" or die "Can't open joint file : $!";
@segments = {};
@segmentScales = {};
@segmentCOMs = {};
my $segmentIndex=0;
my $inSegment=0;
while(<TXT>) {
         if ( m/^beginsegment (.*)$/ ) {
             my $segname = $1;
             $segments[$segmentIndex] = $segname;
             print "begin segment $segname\n";
             $inSegment=1;
             if ($segname eq "ground\r"){
				$segmentScales[$segmentIndex]="0.0, 0.0, 0.0";
				$segmentCOMs[$segmentIndex]="0.0, 0.0, 0.0";
             }
         } 
         elsif ( m/endsegment/ ) {
             print "end segment $segments[$segmentIndex]\n";             
             $inSegment=0;
             $segmentIndex++;
         }
         if (m/scale\s+(\S+)\s+(\S+)\s+(\S+)/){
			if ($inSegment){
				$segmentScales[$segmentIndex]="$1, $2, $3";
				print "scale is $1, $2, $3\n";  
			}           
         }
         if (m/masscenter\s+(\S+)\s+(\S+)\s+(\S+)/){
			if ($inSegment){
				$segmentCOMs[$segmentIndex]="$1, $2, $3";
				print "masscenter is $1, $2, $3\n";  
			}           
         }
}
close TXT;

# The following array handles the mapping between indices in sdfast and those in the jnt file
@mapSegments = (-1, 2, 3, 4, 5, 6, 7, 0, 8, 9, 10, 11, 12, 13, 1, 14, 15);
@sortedSegmentScales = {};
@sortedSegmentCOMs = {};
open FOUT, "> $outFileName" or die "Can't open output file : $!";
print FOUT "static double	scales[16][3] = {\n";
# start at 1 since ground is special
for (my $seg = 0; $seg <=$#segmentScales; $seg++)
{
    my $mappedIndex = $mapSegments[$seg];
    $sortedSegmentScales[$mappedIndex]=$segmentScales[$seg];
}
for (my $seg = 0; $seg <$#segmentScales; $seg++)
{
	if ($seg ==$#segmentScales-1){
		print FOUT "\t{$sortedSegmentScales[$seg]}\n}";
	}
	else{
		print FOUT "\t{$sortedSegmentScales[$seg]},\n";
	}
}
print FOUT ";\n\n";

# Repeat for coms
print FOUT "static double	coms[16][3] = {\n";
for (my $seg = 0; $seg <=$#segmentCOMs; $seg++)
{
    my $mappedIndex = $mapSegments[$seg];
    $sortedSegmentCOMs[$mappedIndex]=$segmentCOMs[$seg];
}
for (my $seg = 0; $seg <$#segmentCOMs; $seg++)
{
	if ($seg ==$#segmentCOMs-1){
		print FOUT "\t{$sortedSegmentCOMs[$seg]}\n}";
	}
	else{
		print FOUT "\t{$sortedSegmentCOMs[$seg]},\n";
	}
}
print FOUT ";";
close FOUT;
