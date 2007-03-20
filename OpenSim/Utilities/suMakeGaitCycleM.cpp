// suMakeGaitCycleM.cpp

// INCLUDES
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Common/rdTools.h>
#include <OpenSim/Common/Storage.h>




using namespace OpenSim;
const int LENGTH = Object_NAME_LENGTH;

//______________________________________________________________________________
/**
 * Reconstruct a full gait cycle from a half gait cycle for a muscle
 * data file. This utility writes two files, one file with the data for the right
 * side and one file with data for the left side.
 * The time data for both files is consistent with a full gait cycle on the 
 * right side.
 *
 * This utility assumes that the input file has columns of right muscle data
 * followed by an equal number of columns of left muscle data -- with NO
 * ligaments.
 *
 * @param argc Number of command line arguments.
 * @param argv Command line arguments:  makeGaitCycleM inFile shiftTime
 * [outFileR] [outFileL]
 */
int main(int argc,char **argv)
{
	// PARSE COMMAND LINE
	char inName[LENGTH],outNameR[LENGTH], outNameL[LENGTH];
	double shiftTime;
	if((argc!=3)&&(argc!=5)) {
		printf("Usage:  makeGaitCycleM inFile shiftTime [outNameR] [outNameL]\n");
		exit(-1);
	} else {
		strcpy(inName,argv[1]);
		sscanf(argv[2],"%lf",&shiftTime);
		if(argc==5) {
			strcpy(outNameR,argv[3]);
                        strcpy(outNameL,argv[4]);
		} else {
			sprintf(outNameR,"%s_ipsi.out",inName);
                        sprintf(outNameL,"%s_contra.out",inName);
		}
	}

	// LOAD DATA
	printf("Loading data from file %s.\n",inName);
	Storage *origStore = new Storage(inName);

	// NUMBER OF COLUMNS
	//int nc = origStore->getSmallestNumberOfStates() - 1;
	int nc = origStore->getSmallestNumberOfStates();
	int nmus;
	if((nc%2)==0) {
		nmus = nc / 2;
	} else {
		nmus = (nc-1) / 2;
	}
	printf("Counted %d muscles.  Assuming left muscles start at index %d.\n",
		nc,nmus);
       
        
     	// RIGHT AND LEFT SIDE COPIES
	Storage *rightStore = new Storage(*origStore,0,nmus);
	Storage *leftAfterStore = new Storage(*origStore,nmus,nmus);
	Storage *leftBeforeStore = new Storage(*origStore,nmus,nmus);
        
	Storage *leftStore = new Storage(*origStore,nmus,nmus);
	Storage *rightAfterStore = new Storage(*origStore,0,nmus);
	Storage *rightBeforeStore = new Storage(*origStore,0,nmus);

	// PREPARE
	StateVector *vec;
	double totalTime = origStore->getLastTime() - origStore->getFirstTime();
        
	// "BEFORE" 
	vec = (StateVector*)
		leftBeforeStore->remove(leftBeforeStore->getFirstEmpty()-1);
	if(vec!=NULL) delete vec;
        vec = (StateVector*)
		rightBeforeStore->remove(rightBeforeStore->getFirstEmpty()-1);
	if(vec!=NULL) delete vec;
        leftBeforeStore->shiftTime(-totalTime);
        rightBeforeStore->shiftTime(-totalTime);
        
	// "MIDDLE"
	vec = (StateVector*)
		rightStore->remove(rightStore->getFirstEmpty()-1);
	if(vec!=NULL) delete vec;
        vec = (StateVector*)
		leftStore->remove(leftStore->getFirstEmpty()-1);
	if(vec!=NULL) delete vec;
                
	// "AFTER"
	vec = (StateVector*)
		leftAfterStore->remove(leftAfterStore->getFirstEmpty()-1);
	if(vec!=NULL) delete vec;
        vec = (StateVector*)
		rightAfterStore->remove(rightAfterStore->getFirstEmpty()-1);
	if(vec!=NULL) delete vec;
        leftAfterStore->shiftTime(totalTime);
        rightAfterStore->shiftTime(totalTime);

	// MAKE OUTPUT
	Storage *outStoreR = new Storage(*rightStore,false);
	outStoreR->append(leftBeforeStore);
	outStoreR->append(rightStore);
	outStoreR->append(leftAfterStore);
	outStoreR->shiftTime(shiftTime);
        
   Storage *outStoreL = new Storage(*leftStore,false);
	outStoreL->append(rightBeforeStore);
	outStoreL->append(leftStore);
	outStoreL->append(rightAfterStore);
	outStoreL->shiftTime(shiftTime);

	// WRITE
	outStoreR->print(outNameR,"w");	
	outStoreL->print(outNameL,"w");

	// CLEANUP
	delete origStore;
	delete leftBeforeStore;
	delete rightStore;
	delete leftAfterStore;
   delete rightBeforeStore;
	delete leftStore;
	delete rightAfterStore;
	delete outStoreR;
        delete outStoreL;

	return(0);
}
