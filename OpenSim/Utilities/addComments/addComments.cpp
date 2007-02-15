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

	string fileName = argv[1];
	string outputFileName = argv[2];
	Object *obj = Object::makeObjectFromFile(fileName);
	std::cout << fileName << " -> " << outputFileName;
	if(!obj) std::cout << "FAILED" << std::endl;
	else {
		std::cout << std::endl;
		IO::SetGFormatForDoubleOutput(true);
		IO::SetPrintOfflineDocuments(false);
		obj->print(outputFileName);
	}
}
