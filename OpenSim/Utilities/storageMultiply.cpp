// storageAdd.cpp

// INCLUDES
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Common/rdTools.h>
#include <OpenSim/Common/Storage.h>

// DEFINES


using namespace OpenSim;
#define MAXLEN 2048

//______________________________________________________________________________
/**
 * Multiplyone storage filebym another.  The output storage is called
 * "storageMultiply.out" unless an output namne is specified on the command line.
 * Output contains state vectors that occur at the same times
 * as the state vectors in the first argument.
 *
 * @param argc Number of command line arguments.
 * @param argv Command line arguments:  storageMultiply file1 file2 [outName]
 */
int main(int argc,char **argv)
{
	// PARSE COMMAND LINE
	char store1Name[MAXLEN],store2Name[MAXLEN],outName[MAXLEN];
	if((argc!=3)&&(argc!=4)) {
		printf("Usage:  storageMultiply store1 store2 [outName]\n");
		exit(-1);
	} else {
		strcpy(store1Name,argv[1]);
		strcpy(store2Name,argv[2]);
		if(argc==4) {
			strcpy(outName,argv[3]);
		} else {
			strcpy(outName,"storageMultiply.out");
		}
	}

	// LOAD DATA
	printf("Loading data from files %s and %s\n",store1Name,store2Name);
	Storage *store1 = new Storage(store1Name);
	Storage *store2 = new Storage(store2Name);

	// ADD
	store1->multiply(store2);

	// WRITE
	store1->print(outName,"w");	

	// CLEANUP
	delete store1;
	delete store2;

	return(0);
}
