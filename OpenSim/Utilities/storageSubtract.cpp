// storageSubtract.cpp

// INCLUDES
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Storage.h>

// DEFINES


using namespace OpenSim;
#define MAXLEN 2048

//______________________________________________________________________________
/**
 * Subtract one storage file from another.  The output storage is called
 * "storageSubtract.out" unless an output namne is specified on the command line.
 * Output contains state vectors that occur at the same times
 * as the state vectors in the first argument.
 *
 * October 8, 2003: Added optional argument to have the algorithm subtract 
 * data with matching row indices rather than matching time (Author:
 * May Liu)
 *
 *
 * @param argc Number of command line arguments.
 * @param argv Command line arguments:  
 *	  storageSubtract file1 file2 [outName] [-i]
 */
int main(int argc,char **argv)
{
	// PARSE COMMAND LINE
	char store1Name[MAXLEN],store2Name[MAXLEN],outName[MAXLEN];
	int iFlag = 0;	

	if((argc!=3)&&(argc!=4)&&(argc!=5)) {
		printf("Usage:  storageSubtract store1 store2 [outName] [-i]\n");
		exit(-1);
	} else {
		strcpy(store1Name,argv[1]);
		strcpy(store2Name,argv[2]);
		if(argc>=4) {
			strcpy(outName,argv[3]);
		} else {
			strcpy(outName,"storageSubtract.out");
		}
		if(argc==5) {
			if(strcmp(argv[4],"-i")!=0) {
				printf("Usage:  storageSubtract store1 store2 [outName] [-i]\n");
				exit(-1);
			} else {
				iFlag = 1;
			}
		}
	}

	// LOAD DATA
	printf("Loading data from files %s and %s\n",store1Name,store2Name);
	Storage *store1 = new Storage(store1Name);
	Storage *store2 = new Storage(store2Name);

	// CHECK TO SEE IF NUMBERS OF ROWS ARE EQUAL
	int size1=0,size2=0,size=0;
	size1 = store1->getSize();
	size2 = store2->getSize();
	size = size1;  if(size2<size) size = size2;
	if(size1!=size2) {
		printf("WARNING:	Storages have different number of rows.\n");
	}

	// SUBTRACT
	if(iFlag==0) {
		store1->subtract(store2);
		
	} else {
		int i;
		StateVector *vec1,*vec2;
		store1->reset(size-1);
		for(i=0;i<size;i++) {
			vec1 = store1->getStateVector(i);
			vec2 = store2->getStateVector(i);
			vec1->subtract(vec2);
		}
	}

	// PRINT
	store1->print(outName,"w");

	// CLEANUP
	delete store1;
	delete store2;

	return(0);
}
