// rdConvertControls.cpp
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
#include <string>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Storage.h>
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
	const Array<std::string> &colLabels = storage->getColumnLabels();
	int startIndex = colLabels.findIndex(columnName)-1;
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

	// Try printing to a different file without copying
	controlSet.clearXMLStructures();
	controlSet.print("New_"+outName);
	return(0);
}



ControlLinear*
ExtractControl(Storage* storage,int index)
{
	int i;

	// NAME ATTRIBUTE
	const Array<std::string> &columnLabels = storage->getColumnLabels();
	std::string colName = columnLabels.get(index+1);

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
	ControlLinear *control = new ControlLinear;
	control->setName(colName);
	control->clearControlNodes();

	// APPEND CONTROL ELEMENTS
	int n = nTimes;
	if(n>nValues) n = nValues;
	for(i=0;i<n;i++) {
		control->setControlValue(times[i],values[i]);
		//ControlLinearNode *node;
		//char nodeName[256];
		//sprintf(nodeName,"%d",i);
		//node->setName(nodeName);
	}

	// CHECK
	cout<<"Control "<<control->getName()<<" has "<<control->getNumParameters()<<" nodes."
		<<endl;


	return(control);
}
