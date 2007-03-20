// storageInterpolate.cpp

// INCLUDES
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Common/Storage.h>

// DEFINES


using namespace OpenSim;
#define MAXLEN 2048

//______________________________________________________________________________
/**
 * Interpolate a storage instance.  The output storage is called
 * "storageInterpolate.out" unless an output name is specified on the command
 * line.  Output contains state vectors that occur with the initial and final
 * times specified, with the specified time step.
 *
 * @param argc Number of command line arguments.
 * @param argv String array of command line arguments.
 */
int main(int argc,char **argv)
{
	// PARSE COMMAND LINE
	char storeFileName[MAXLEN],outFileName[MAXLEN];
	double startTime, endTime, stepSize;
	bool writeSIMMHeader;
	if((argc!=6)&&(argc!=7)) {
		printf("Usage:  storageInterpolate storeFilename startTime endTime timeStepSize writeSIMMHeaderOrNot [outFileName]\n");
		exit(-1);
	} else {
		strcpy(storeFileName,argv[1]);
		startTime = 0.5;
		endTime = 1.989;
		stepSize = 0.001;
		writeSIMMHeader = true;
		if(argc==7) {
			strcpy(outFileName,argv[6]);
		} else {
			strcpy(outFileName,"storageInterpolate.out");
		}
	}

	// LOAD DATA
	printf("Loading data from file %s\n",storeFileName);
	Storage *store = new Storage(storeFileName);

	// INTERPOLATE
	// Add startTime node and endTime node if each doesn't already exist

	// WRITE
	store->setWriteSIMMHeader(writeSIMMHeader);
	store->print(outFileName,"w");	

	// CLEANUP
	delete store;

	return(0);
}
