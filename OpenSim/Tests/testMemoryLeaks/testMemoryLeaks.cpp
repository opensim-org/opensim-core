#include <iostream>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <windows.h>
#include <psapi.h>

using namespace OpenSim;
using namespace std;

unsigned int Memory_Usage()
{
    HANDLE process=GetCurrentProcess();
    PROCESS_MEMORY_COUNTERS counters;
    GetProcessMemoryInfo(process,&counters,sizeof(counters));
    return (unsigned int)counters.WorkingSetSize;
}

int main(int argc,char **argv)
{
	LoadOpenSimLibrary("osimActuators");
	LoadOpenSimLibrary("osimSimbodyEngine");
	LoadOpenSimLibrary("osimSimmKinematicsEngine");

	std::string filename = "fullbody_test/FullBody_markerchange.osim";
	if(argc>1) filename=argv[1];

	std::cout << "START " << Memory_Usage() << std::endl;

	Model *originalModel = new Model(filename);
	originalModel->setup();

	std::cout << "BASELINE " << Memory_Usage() << std::endl;

	try {
	for(int i=0; i<3; i++) {
		std::cout << std::endl;
		std::cout << i << " BEFORE " << Memory_Usage() << std::endl;
		Model *model = new Model(*originalModel);
		std::cout << i << " AFTER LOAD " << Memory_Usage() << std::endl;
		model->setup();
		std::cout << i << " AFTER SETUP " << Memory_Usage() << std::endl;
		delete model;
		std::cout << i << " AFTER DELETE " << Memory_Usage() << std::endl;
	}

	delete originalModel;
	std::cout << "AFTER FINAL DELETE " << Memory_Usage() << std::endl;

	} catch (Exception &ex) {
		ex.print(std::cout);
	}
}
