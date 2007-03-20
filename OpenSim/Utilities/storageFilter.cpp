// storageSubtract.cpp

// INCLUDES
#include <iostream>
#include <string>
#include <OpenSim/Common/Storage.h>

// DEFINES


using namespace OpenSim;
#define MAXLEN 2048

using namespace std;

static void PrintUsage(ostream &aOStream);

//______________________________________________________________________________
/**
 * Filter the columns in a storage file.
 *
 * @param argc Number of command line arguments.
 * @param argv Command line arguments:  
 *	  storageSubtract file1 file2 [outName] [-i]
 */
int main(int argc,char **argv)
{
	if(argc<3) {
		PrintUsage(cout);
		return(0);
	}

	// PARSE COMMAND LINE
	int i;
	string option;
	string storeName;
	string outName="filtered.sto";
	double lowpassFIR = -1.0;
	for(i=1;i<argc;i++) {

		option = argv[i];

		// USAGE
		if((option=="-help")||(option=="-Help")||(option=="-H")) {
			PrintUsage(cout);
			return(0);

		// STORAGE FILE
		} else if((option=="-File")||(option=="-F")) {\
			if(argc>(i+1)) {
				storeName = argv[i+1];
				++i;
			}

		// OUTPUT NAME
		} else if((option=="-Out")||(option=="-O")) {
			if(argc>(i+1)) {
				outName = argv[i+1];
				++i;
			}

		// LOWPASS FIR
		} else if((option=="-LowpassFIR")||(option=="-LFIR")) {
			if(argc>(i+1)) {
				sscanf(argv[i+1],"%lf",&lowpassFIR);
				++i;
			}

		// UNRECOGNIZED
		} else {
			cout<<"Unrecognized option: "<<argv[i]<<endl;
		}
	}

	// ERROR CHECK
	if(storeName=="") {
		cout<<"\nA storage file must be specified using the -File option.\n\n";
		return(-1);
	}

	// LOAD STORAGE FILE
	cout<<"Loading storage file "<<storeName<<"...\n";
	Storage *store = new Storage(storeName.c_str());

	// FILTER
	// Lowpass FIR
	if(lowpassFIR>=0.0) {
		cout<<"\nLowpass FIR filtering using a cutoff frequency of "<<lowpassFIR<<".\n";
		store->pad(60);
		store->lowpassFIR(50,lowpassFIR);
	}

	// OUTPUT
	store->print(outName.c_str(),"w");

	// CLEANUP
	delete store;

	return(0);
}

//_____________________________________________________________________________
/**
 * Print the usage for this application
 */
void PrintUsage(ostream &aOStream)
{
	aOStream<<"\n\nstorageFilter.exe:\n\n";
	aOStream<<"Option              Argument         Action / Notes\n";
	aOStream<<"------              --------         --------------\n";
	aOStream<<"-Help, -H                            Print the command-line options.\n";
	aOStream<<"-File, -F           StorageFile      Specifies the name of the storage file to filter.\n";
	aOStream<<"-Out, -O            OutputFile       Specifies the name of the file to which to write the filtered data.\n";
	aOStream<<"-LowpassFIR, -LFIR  Cutoff           Specifiy the lowpass cutoff frequency for an FIR filter.\n";
	aOStream<<"\n\n";
}


