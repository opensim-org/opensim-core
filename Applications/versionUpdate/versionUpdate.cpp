// versionUpdate.cpp
// Author: Ayman Habib
/*
* Copyright (c)  2011, Stanford University. All rights reserved. 
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

// INCLUDE
#include <OpenSim/OpenSim.h>  // clock(), clock_t, CLOCKS_PER_SEC
#include <OpenSim/Common/IO.h>
#include <OpenSim/version.h>
using namespace OpenSim;
using namespace std;

static void PrintUsage(const char *aProgName, ostream &aOStream);

//_____________________________________________________________________________
/**
 * Main routine to read an object and write it to the same object with latest XML format.
 */
int main(int argc,char **argv)
{
	//----------------------
	// Surrounding try block
	//----------------------
	try {

	// PARSE COMMAND LINE
	string option = "";
	string inputFileName = "";
	string outputFileName = "";
	if(argc<2) {
		PrintUsage(argv[0], cout);
		return(-1);
	}
	inputFileName = string(argv[1]);
	outputFileName = (argc == 2)?(outputFileName=inputFileName): string(argv[2]);
	// ERROR CHECK
	if(inputFileName=="") {
		cout<<"\n\versionUpdate.exe: ERROR- An input file must be specified.\n";
		PrintUsage(argv[0], cout);
		return(-1);
	}
	
	string::size_type extSep = inputFileName.rfind(".");

	if (extSep == string::npos) {
		cout<<"\n\versionUpdate.exe: ERROR- Unknown file type encontered. File extension must be specified.\n";
		PrintUsage(argv[0], cout);
		return 1;// if '_fileName' contains path information...
	}
	std::string extension  = inputFileName.substr(extSep);
	if (extension == ".sto"){
		Storage stg(inputFileName);
		stg.print(outputFileName);
		return (0);
	}
	if (extension != ".xml" && extension != ".osim") {
		cout<<"\n\versionUpdate.exe: ERROR- Unknown file type encontered. Only .xml, .osim and .sto files are supported.\n";
		PrintUsage(argv[0], cout);
		return 1;// if '_fileName' contains path information...
	}

	Object* newObject = Object::makeObjectFromFile(inputFileName);

	newObject->clone()->print(outputFileName);

	//----------------------------
	// Catch any thrown exceptions
	//----------------------------
	} catch(const Exception& x) {
		x.print(cout);
		return(-1);
	}
	//----------------------------
	return(0);
}


//_____________________________________________________________________________
/**
 * Print the usage for this application
 */
void PrintUsage(const char *aProgName, ostream &aOStream)
{
	string progName=IO::GetFileNameFromURI(aProgName);
	aOStream<<"\n\n"<<progName<<":\n"<<GetVersionAndDate()<<" inputFile outputFile\n\n";
	aOStream<<"Option              Argument         Action / Notes\n";
	aOStream<<"------              --------         --------------\n";
	aOStream<<"inputFileName		Specify the name of the OpenSim xml file (.xml, .osim) to perform versionUpdate on.\n";
	aOStream<<"outputFileName		Specify the name of the output file.\n";
}

