#include <string.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

using namespace OpenSim;
using namespace std;

int main(int argc,char **argv)
{
	LoadOpenSimLibrary("osimTools");
	LoadOpenSimLibrary("osimActuators");
	LoadOpenSimLibrary("osimAnalyses");
	LoadOpenSimLibrary("osimSimulation");
	LoadOpenSimLibrary("osimSimmKinematicsEngine");

	int offset=0;
	if(argc>1 && string(argv[1])=="-offline") offset++;
	else IO::SetPrintOfflineDocuments(false);
	if(argc<1+offset+2) {
		std::cerr << "Not enough arguments: <INPUT_FILE> <OUTPUT_FILE>" << std::endl;
		exit(1);
	}
	string fileName = argv[offset+1];
	string outputFileName = argv[offset+2];
	Object *obj = Object::makeObjectFromFile(fileName);
	std::cout << fileName << " -> " << outputFileName;
	if(!obj) std::cout << " FAILED" << std::endl;
	else {
		std::cout << std::endl;
		IO::SetGFormatForDoubleOutput(true);
		obj->print(outputFileName);
	}
}
