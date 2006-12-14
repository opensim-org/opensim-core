// perturb.cpp
// author:  Frank C. Anderson

// INCLUDE
#include <string>
#include <iostream>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Simulation/Model/LoadModel.h>




using namespace OpenSim;
using namespace std;

//_____________________________________________________________________________
/**
 * Main routine for conducting a perturbation analysis.
 */
int main(int argc,char **argv)
{

	// INTERPRET COMMAND LINE
	/*
	string setupFileName = "";
	string modelName = "";
	if(argc<=2) {
		cout<<"\n\nusage: perturb.exe setupFile.xml modelName\n\n";
		exit(-1);
	} else {
		setupFileName = argv[1];
		modelName = argv[2];
	}

	// CONSTRUCT INVESTIGATION
	cout<<"Loading perturbation setup file "<<setupFileName<<".\n\n";
	InvestigationPerturbation perturb(setupFileName);
	perturb.print("check.xml");
	*/

#if 0 
	// commented out because LoadModel changed
 
	// LOAD MODEL
	AbstractModel *model = LoadModel(argc,argv);
	cout<<"Finished call to LoadModel\n";
	if(model==NULL) {
		cout<<"\nperturb:  ERROR- failed to load model.\n";
		exit(-1);
	}
	cout<<"-----------------------------------------------------------------------\n";
	cout<<"Loaded library\n";
	cout<<"-----------------------------------------------------------------------\n";
	model->printBasicInfo(cout);
	cout<<"-----------------------------------------------------------------------\n\n";

	// RUN
	cout<<"Running..."<<endl<<endl;
	//perturb.setModel(model);
	//perturb.run();

#endif

	return(0);
}

