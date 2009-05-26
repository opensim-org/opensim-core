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
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/XMLNode.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>




using namespace OpenSim;
using namespace std;


// DECLARATIONS
ControlLinear*
ExtractControl(DOMElement *aElmt);


//______________________________________________________________________________
/**
 * Convert a controls file from the old XML format to the new XML format.
 *
 * @param argc Number of command line arguments (should be 2).
 * @param argv Command line arguments:  convertControls inFile [outFile]
 */
int main(int argc,char **argv)
{
	// PARSE COMMAND LINE
	string inName,outName;
	if((argc!=2)&&(argc!=3)) {
		printf("Usage:  convertControls inFile [outFile]\n");
		exit(-1);
	} else {
		inName = argv[1];
		if(argc==3) {
			outName = argv[2];
		} else {
			outName = "convertControls.out";
		}
	}

	// OUTPUT PRECISION
	IO::SetPrecision(16);
	IO::SetDigitsPad(-1);

	// REGISTER TYPES
	//Object::RegisterType(ControlLinear());
	//Object::RegisterType(ControlLinearNode());

	// CREATE AN EMPTY CONTROL SET
	ControlSet controlSet;

	// LOAD OLD XML DOCUMENT
	cout<<"Loading file "<<inName<<" in old XML format..."<<endl;
	XMLDocument *doc = new XMLDocument(inName);
	DOMDocument *dom = doc->getDOMDocument();
	if(dom==NULL) {
		cout<<"Unable to load document "<<inName<<"."<<endl;
		exit(-1);
	}

	// GET ROOT NODE
	DOMElement *root = dom->getDocumentElement();


	// GET LIST OF CONTROL NODES
	unsigned int j;
	string type = "ControlLinear";
	XMLCh *tagName = XMLString::transcode(type.c_str());
	DOMNodeList *list = root->getElementsByTagName(tagName);
	delete[] tagName;

	// LOOP THROUGH LIST
	DOMElement *elmt;
	ControlLinear *controlNode;
	for(j=0;j<list->getLength();j++) {

		// GET ELEMENT
		elmt = (DOMElement*) list->item(j);

		// EXTRACT CONTROL NODE
		controlNode = ExtractControl(elmt);

		// APPEND ON TO CONTROL SET
		controlSet.append(controlNode);
	}


	// WRITE CONTROL SET TO FILE
	controlSet.print(outName);

	return(0);
}



ControlLinear*
ExtractControl(DOMElement *aElmt)
{
	if(aElmt==NULL) return(NULL);


	// NAME ATTRIBUTE
	string elmtName = XMLNode::GetAttribute(aElmt,"name");
	cout<<"\nFound control node "<<elmtName<<endl;


	// TIME
	int nTimes;
	double *times;
	DOMElement *elmt = XMLNode::GetFirstChildElementByTagName(aElmt,"time");
	nTimes = XMLNode::GetDblArray(elmt,times);
	cout<<"nTimes = "<<nTimes<<", ";

	// VALUE
	int nValues;
	double *values;
	elmt = XMLNode::GetFirstChildElementByTagName(aElmt,"value");
	nValues = XMLNode::GetDblArray(elmt,values);
	cout<<"nValue = "<<nValues<<", ";

	// MIN
	int nMins;
	double *mins;
	elmt = XMLNode::GetFirstChildElementByTagName(aElmt,"min");
	nMins = XMLNode::GetDblArray(elmt,mins);
	cout<<"nMins = "<<nMins<<", ";

	// MAX
	int nMaxs;
	double *maxs;
	elmt = XMLNode::GetFirstChildElementByTagName(aElmt,"max");
	nMaxs = XMLNode::GetDblArray(elmt,maxs);
	cout<<"nMaxs = "<<nMaxs<<endl;

	// CONSTRUCT LINEAR CONTROL NODE
	ControlLinear *control = new ControlLinear;
	control->setName(elmtName);
	control->clearControlNodes();

	// APPEND CONTROL ELEMENTS
	for(int i=0;i<min(nTimes,nValues);i++)
		control->setControlValue(times[i],values[i]);

	for(int i=0;i<min(nTimes,nMins);i++)
		control->setControlValueMin(times[i],mins[i]);

	for(int i=0;i<min(nTimes,nMaxs);i++)
		control->setControlValueMax(times[i],maxs[i]);

	// CHECK
	cout<<"Control "<<control->getName()<<" has "<<control->getNumParameters()<<" nodes."
		<<endl;


	return(control);
}
