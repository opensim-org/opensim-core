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

	// LOAD DATA
	printf("Loading data from files\n");
	rdSimStorage *s1 = new rdSimStorage("s1.txt");
	rdSimStorage *s2 = new rdSimStorage("s2.txt");
	rdSimStorage *s3 = new rdSimStorage("s3.txt");
	rdSimStorage *s4 = new rdSimStorage("s4.txt");
	rdSimStorage *s5 = new rdSimStorage("s5.txt");

	// CONSTRUCT SAMPLED SERIES
	rdSimStorage *ave = new rdSimStorage(*s1);
	ave->reset();
	ave->setName("AverageS1S2S3S4S5");

	// NOW SAMPLE DATA AT THE CORRECT TIMES
	int i,n=0;
	double t,*y=NULL,*y1=NULL,*y2=NULL,*y3=NULL,*y4=NULL,*y5=NULL;
	for(t=0.0;t<=1.0;t+=0.01) {

		// GET THE SUBJECT DATA
		n = s1->getStates(t,n,&y1);
		n = s2->getStates(t,n,&y2);
		n = s3->getStates(t,n,&y3);
		n = s4->getStates(t,n,&y4);
		n = s5->getStates(t,n,&y5);

		// ALLOCATE Y
		if(y==NULL) {
			y = new double[n];
		}

		// CHECK FOR NO DATA
		if(y1[1]<=0) { y1[0]=0.0;  y1[1]=0.0;  y1[2]=0.0; }
		if(y2[1]<=0) { y2[0]=0.0;  y2[1]=0.0;  y2[2]=0.0; }
		if(y3[1]<=0) { y3[0]=0.0;  y3[1]=0.0;  y3[2]=0.0; }
		if(y4[1]<=0) { y4[0]=0.0;  y4[1]=0.0;  y4[2]=0.0; }
		if(y5[1]<=0) { y5[0]=0.0;  y5[1]=0.0;  y5[2]=0.0; }

		// AVERAGE
		for(i=0;i<n;i++) {
			y[i] = (y1[i] + y2[i] + y3[i] + y4[i] + y5[i])/5.0;
		}

		// APPEND
		ave->append(100.0*t,n,y);
	}

	// WRITE DATA
	ave->print("s.txt","w");	

	// CLEANUP
	delete[] y;
	delete s1;
	delete ave;

	return(0);
}
