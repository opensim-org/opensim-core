// rdConvertControls.cpp

// INCLUDES
#include <string>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>




using namespace OpenSim;
using namespace std;


// DECLARATIONS
ControlLinear*
ExtractControl(Storage* storage, int index);


//______________________________________________________________________________
/**
 * Convert a controls file from storage to the new XML format.
 *
 * @param argc Number of command line arguments (should be 2).
 * @param argv Command line arguments:  convertControls inFile [outFile]
 */
int main(int argc,char **argv)
{
	// PARSE COMMAND LINE
	string inName,outName;
	int startIndex=0;
	int numControls=0;
	char *columnName;
	if((argc!=4)&&(argc!=5)) {
		printf("Usage:  convertControls inFile columnName numControls [outFile]\n");
		exit(-1);
	} else {
		inName = argv[1];
		columnName = argv[2];
		sscanf(argv[3],"%d",&numControls);
		if(argc==5) {
			outName = argv[4];
		} else {
			outName = "convertControls.out";
		}
	}

	// OUTPUT PRECISION
	IO::SetPrecision(16);
	IO::SetDigitsPad(-1);

	// REGISTER TYPES
	Object::RegisterType(ControlLinear());
	Object::RegisterType(ControlLinearNode());

	// CREATE AN EMPTY CONTROL SET
	ControlSet controlSet;

	// LOAD RDSTORAGE DOCUMENT
	cout<<"Loading storage "<<inName<<" to be converted..."<<endl;
	Storage* storage = new Storage(inName.c_str());

	// DETERMINE START INDEX
	int i;
	const char *colLabels = storage->getColumnLabels();
	char *labels = new char[strlen(colLabels)+1];
	strcpy(labels,colLabels);
	cout<<endl<<endl<<labels<<endl<<endl;
	char *tok = strtok(labels," \t");
	for(i=0;(tok=strtok(NULL," \t"));i++) {

		if(strcmp(tok,columnName)==0) break;
	}
	startIndex = i;
	cout<<"startIndex = "<<startIndex<<endl;



		// LOOP THROUGH LIST
	int j;
	ControlLinear *control;
	for(j=0;j<numControls;j++) {

		// EXTRACT CONTROL NODE
		control = ExtractControl(storage,j+startIndex);

		// APPEND ON TO CONTROL SET
		controlSet.append(control);
	}


	// WRITE CONTROL SET TO FILE
	controlSet.print(outName);

	return(0);
}



ControlLinear*
ExtractControl(Storage* storage,int index)
{

	// NAME ATTRIBUTE
	const char *columnLabels = storage->getColumnLabels();
	//	cout<<"these are the column labels\n "<<columnLabels<<endl;
	char *labels = new char[(strlen(columnLabels) +2)];
	strcpy(labels,columnLabels);
	char *colName;
	const char delim[] = "\t ";
	int i;
	colName = strtok(labels,delim);
	for(i=0;i<(index+1);i++)
		colName = strtok(NULL,delim);


	// TIME
	cout<<"\nExtracting column "<<colName<<"..."<<endl;
	double *times = NULL;
	int nTimes = storage->getTimeColumn(times);
	cout<<"nTimes = "<<nTimes<<", ";

	// VALUE
	int nValues = nTimes;
	int rValue;
	double *values = NULL;
	rValue = storage->getDataColumn(index,values);
	cout<<"nValue = "<<nValues<<", ";

	// CONSTRUCT LINEAR CONTROL NODE
	ControlLinear *control = new ControlLinear(NULL,colName);
	ArrayPtrs<ControlLinearNode> &nodes = control->getNodeArray();
	nodes.setSize(0);

	// APPEND CONTROL ELEMENTS
	int n = nTimes;
	if(n>nValues) n = nValues;
	ControlLinearNode *node;
	PropertySet *propSet;
	Property *prop;
	char nodeName[256];
	for(i=0;i<n;i++) {
		node = new ControlLinearNode(times[i],values[i]);
		sprintf(nodeName,"%d",i);
		node->setName(nodeName);
		propSet = &node->getPropertySet();
		prop = propSet->get("min");
		prop->setUseDefault(true);
		prop = propSet->get("max");
		prop->setUseDefault(true);
		nodes.append(node);
	}

	// CHECK
	cout<<"Control "<<control->getName()<<" has "<<nodes.getSize()<<" nodes."
		<<endl;

	// CLEAN UP
	delete[] labels;


	return(control);
}
