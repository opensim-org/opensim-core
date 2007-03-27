// perturbStorage.cpp

// INCLUDES
#include <string>
#include <iostream>
#include <OpenSim/Common/Storage.h>




using namespace OpenSim;
using namespace std;


// DEFINES
#define MAXLEN 2048

//______________________________________________________________________________
/**
 * Read in a storage file, perturb one of its columns of data, and write
 * the data back out to file.
 *
 * @param argc Number of command line arguments.
 * @param argv Command line arguments:  storageAdd file1 file2 [outName]
 */
int main(int argc,char **argv)
{

	// PARSE COMMAND LINE
	char inName[MAXLEN],columnName[MAXLEN];
	double startTime,endTime,amount;
	if(argc!=6) {
		cout<<endl<<"Usage:"<<endl;
		cout<<"perturbStorage inFile columnName startTime endTime amount"<<endl;
		cout<<endl<<endl;
		exit(-1);
	} else {
		sscanf(argv[1],"%s",inName);
		sscanf(argv[2],"%s",columnName);
		sscanf(argv[3],"%lf",&startTime);
		sscanf(argv[4],"%lf",&endTime);
		sscanf(argv[5],"%lf",&amount);
	}

	// LOAD DATA
	cout<<"\nLoading data from file "<<inName<<endl<<endl;
	Storage store(inName);


	// GET COLUMN LABELS
	const Array<std::string> &columnLabels = store.getColumnLabels();
	int column = columnLabels.findIndex(columnName);
	if(column < 0) {
		cout<<endl<<endl<<"Column "<<columnName<<" not found.\n"<<endl<<endl;
		exit(-1);
	} else if(column==0) {
		// UNSUPPORTED COLUMN
		cout<<"Perturbing the first column (col=0) is not currently supported."<<endl;
		exit(-1);
	}


	// PERTURBING COLUMN
	cout<<"Perturbing column "<<column<<endl<<endl;
	StateVector *vec;
	double t;
	int n = store.getSize();
	for(int i=0;i<n;i++) {

		vec = store.getStateVector(i);
		if(vec==NULL) continue;

		t = vec->getTime();
		if(t<startTime) continue;
		if(t>endTime) break;

		vec->add(column-1,amount);
	}

	// WRITE
	char outName[MAXLEN];
	char *nameRoot = strtok(inName,".");
	strcpy(outName,nameRoot);
	strcat(outName,"_");
	strcat(outName,columnName);
	strcat(outName,"_");
	strcat(outName,argv[3]);
	strcat(outName,"_");
	strcat(outName,argv[4]);
	strcat(outName,"_");
	strcat(outName,argv[5]);
	strcat(outName,".mot");

	cout<<"Writing perturbed data to file "<<outName<<endl;
	cout<<endl<<endl;
	store.print(outName,"w");

	return(0);
}
