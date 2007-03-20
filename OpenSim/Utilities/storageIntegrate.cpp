// storageIntegrate.cpp

// INCLUDES
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/IO.h>

// DEFINES


using namespace OpenSim;
#define MAXLEN 2048

//______________________________________________________________________________
/**
 * Integrate a storage instance.
 *
 * @param argc Number of command line arguments.
 * @param argv Command line arguments:  storageIntegrate file [outName]
 * [ti] [tf]
 */
int main(int argc,char **argv)
{
	IO::SetPrecision(16);

	// PARSE COMMAND LINE
	char storeName[MAXLEN],outName[MAXLEN];
	double tStart,tEnd;
	if((argc!=2)&&(argc!=3)&&(argc!=5)) {
		printf("Usage:  storageIntegrate file [outName] [ti] [tf]\n");
		exit(-1);
	} else {
		strcpy(storeName,argv[1]);
		if((argc==3)||(argc==5)) {
			strcpy(outName,argv[2]);
		} else {
			strcpy(outName,"storageIntegrate.out");
		}
		if(argc==5){
			sscanf(argv[3],"%lf",&tStart);
			sscanf(argv[4],"%lf",&tEnd);
		}
	}

	// LOAD DATA
	printf("Loading data from file %s.\n",storeName);
	Storage *store = new Storage(storeName);

	// GET START AND END INTEGRATION TIMES IF NONE WERE ENTERED
	if(argc!=5){
		tStart = store->getFirstTime();
		tEnd = store->getLastTime();
	}

	// INTEGRATE
	Storage *result = store->integrate(tStart,tEnd);
	
	// WRITE
	result->print(outName);	

	// CLEANUP
	delete store;
	delete result;

	return(0);
}
