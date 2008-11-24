// storageSubtract.cpp
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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
	double lowpassIIR = -1.0;
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

		// LOWPASS IIR
		} else if((option=="-LowpassIIR")||(option=="-LIIR")) {
			if(argc>(i+1)) {
				sscanf(argv[i+1],"%lf",&lowpassIIR);
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

	if(lowpassIIR>=0.0) {
		cout<<"\nLowpass IIR filtering using a cutoff frequency of "<<lowpassIIR<<".\n";
		store->pad(store->getSize()/2);
		store->lowpassIIR(lowpassIIR);
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
	aOStream<<"-LowpassIIR, -LIIR  Cutoff           Specifiy the lowpass cutoff frequency for an IIR/Butterworth filter.\n";
	aOStream<<"The options IIR and FIR are mutually exclusive.\n";
	aOStream<<"\n\n";
}


