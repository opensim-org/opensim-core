// sample.cpp


//==============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <rdSimUtility.h>


//______________________________________________________________________________
/**
 * Sample a data series at the same times as a reference data series.
 */
int main(int argc,char **argv)
{

	// PARSE FILE NAME
	char refName[2048],dataName[2048];
	if(argc!=3) {
		printf("Usage:  sample refSeries dataSeries\n");
		exit(-1);
	} else {
		strcpy(refName,argv[1]);
		strcpy(dataName,argv[2]);
	}

	// LOAD DATA
	printf("Loading data from files %s and %s\n",refName,dataName);
	rdSimStorage *ref = new rdSimStorage(refName);
	rdSimStorage *data = new rdSimStorage(dataName);

	// CONSTRUCT SAMPLED SERIES
	rdSimStorage *sampled = new rdSimStorage(*data);
	sampled->reset();

	// NOW SAMPLE DATA AT THE CORRECT TIMES
	int i,n=0;
	double t,*y=NULL;
	rdSimVector *vec=NULL;
	for(i=0;i<ref->getFirstEmpty();i++) {
		vec = ref->getStates(i);
		t = vec->getTime();
		n = data->getStates(t,n,&y);
		sampled->append(t,n,y);
	}

	// WRITE DATA
	strcat(dataName,"_sampled");
	sampled->print(dataName,"w");	

	// CLEANUP
	delete[] y;
	delete ref;
	delete data;
	delete sampled;

	return(0);
}
