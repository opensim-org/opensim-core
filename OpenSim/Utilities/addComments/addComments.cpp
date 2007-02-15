#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/VisibleObject.h>
#include <OpenSim/Subject/SimmSubject.h>
#include <OpenSim/Applications/IK/IKTool.h>
#include <OpenSim/CMC/CMCTool.h>
#include <OpenSim/Analyses/ForwardTool.h>
#include <OpenSim/Analyses/PerturbationTool.h>

using namespace OpenSim;
using namespace std;

int main(int argc,char **argv)
{
	Object::RegisterType(VisibleObject());
	Object::RegisterType(SimmSubject());
	SimmSubject::registerTypes();
	IKTool::registerTypes();
	Object::RegisterType(IKTool());
	Object::RegisterType(CMCTool());
	Object::RegisterType(ForwardTool());
	Object::RegisterType(PerturbationTool());

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
	if(!obj) std::cout << "FAILED" << std::endl;
	else {
		std::cout << std::endl;
		IO::SetGFormatForDoubleOutput(true);
		obj->print(outputFileName);
	}
}
