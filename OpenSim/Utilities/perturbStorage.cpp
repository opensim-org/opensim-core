// perturbStorage.cpp

// INCLUDES
#include <string>
#include <iostream>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Storage.h>




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
	const char *columnLabels = store.getColumnLabels();
	char *label,*labels = new char[strlen(columnLabels)+1];
	strcpy(labels,columnLabels);

	// PARSE TO DESIRED COLUMN
	int i,column;
	bool notFound;
	for(i=0,notFound=true;notFound;i++) {
		
		if(i==0) {
			label = strtok(labels," ");
		} else {
			label = strtok(NULL," ");
		}

		if(label==NULL) break;

		cout<<label<<endl;
		
		if(strcmp(label,columnName)==0) {
			column = i;
			notFound = false;
		}
	}


	// NOT FOUND
	if(notFound) {
		cout<<endl<<endl<<"Column "<<columnName<<" not found.\n"<<endl<<endl;
		exit(-1);
	}
	// UNSUPPORTED COLUMN
	if(column<=0) {
		cout<<"Perturbing the first column (col=0) is not currently supported."<<endl;
		exit(-1);
	}


	// PERTURBING COLUMN
	cout<<"Perturbing column "<<column<<endl<<endl;
	StateVector *vec;
	double t;
	int n = store.getSize();
	for(i=0;i<n;i++) {

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
