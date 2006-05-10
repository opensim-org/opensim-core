// convert.cpp
// Convert wcontrols into an ControlsLinear file

#include <stdlib.h>
#include <stdio.h>

const int NN = 15;
const int NM = 54;
const int NMHALF = NM/2;

//______________________________________________________________________________
/**
 * Main
 */
int main(int argc,char *argv[])
{

	// OPEN FILE
	FILE *fp = fopen("wcontrols","r");
	
	// NODE TIMES
	int i,j;
	double t[NN];
	printf("reading tnode...\n");
	for(i=0;i<NN;i++) fscanf(fp,"%lf",&t[i]);

	// INITIAL MUSCLE ACTIVATIONS
	double a[NM];
	for(i=0;i<NM;i++) fscanf(fp,"%lf",&a[i]);

	// MUSCLE EXCITATIONS
	double x[NM][NN];
	for(i=0;i<NN-1;i++) {
		printf("reading node %d\n",i);
		for(j=0;j<NM;j++) {
			fscanf(fp,"%lf",&x[j][i]);
		}
	}

	// CLOSE FILE
	fclose(fp);


	// WRITE NEW FORMAT
	fp = fopen("wcontrolsOptimal.txt","w");

	// HEADER
	fprintf(fp,"OptimalGaitControlsForUTModel\n");
	fprintf(fp,"%d\n",NM);

	// MUSCLE CONTROLS
	for(i=0;i<NM;i++) {

		printf("muscle %d\n",i);

		// TYPE
		fprintf(fp,"LINEAR 400\n");

		// NAME
		fprintf(fp,"Control%d\n",i);

		// NUMBER OF NODES
		fprintf(fp,"%d\n",NN);

		// TIME/VALUES
		for(j=0;j<NN-1;j++) {
			fprintf(fp,"%24.16lf %24.16lf\n",t[j],x[i][j]);
		}
		if(i<NMHALF) {
			fprintf(fp,"%24.16lf %24.16lf\n",t[NN-1],x[i+NMHALF][0]);
		} else {
			fprintf(fp,"%24.16lf %24.16lf\n",t[NN-1],x[i-NMHALF][0]);
		}
	}

	// CLOSE
	fclose(fp);

	return(0);
}
