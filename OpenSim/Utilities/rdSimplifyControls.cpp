// rdSimplifyControls.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

// INCLUDES
#include <string>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertySet.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Simulation/Control/Control.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>




using namespace OpenSim;
using namespace std;


//______________________________________________________________________________
/**
 * Simplify a the controls in a controls file.\n\n
 *
 * The user must specify and input file as well as a cutoff frequency and
 * a distance.  The cutoff frequency is used to low-pass filter the
 * original control curves in the input file.  The distance use used
 * to specify the distance error allowed between the simplified curve and
 * the low-pass filtered curve.\n\n
 *
 * The new control set is written to controls_simple.ctr.\n
 * For the purpose of checking the quality of the simplification, the
 * control set is also written to two storage files:  controls_original.sto,
 * which contains the original controls, and controls_simple.sto, which
 * contains the simplified controls.  These .sto files can be plotted
 * using Excel, for example.
 *
 * @param argc Number of command line arguments.
 * @param argv Command line arguments:\n
 * simplifyControls inFile cutoff_frequency distance
 */
int main(int argc,char **argv)
{
	// PARSE COMMAND LINE
	string inName,outName;
	double cutoff,distance;
	if(argc!=4) {
		cout<<"\n\nUsage: simplifyControls ";
		cout<<"inFile cutoff_frequency distance\n\n";
		exit(-1);
	} else {
		inName = argv[1];
		sscanf(argv[2],"%lf",&cutoff);
		sscanf(argv[3],"%lf",&distance);
	}

	// OUTPUT PRECISION
	IO::SetPrecision(16);
	IO::SetDigitsPad(-1);

	// REGISTER TYPES
	Object::RegisterType(ControlConstant());
	Object::RegisterType(ControlLinear());
	Object::RegisterType(ControlLinearNode());

	// LOAD CONTROL SET
	cout<<"Loading control set "<<inName<<" to be simplified..."<<endl;
	ControlSet controlSet(inName);

	// EXTRACT INITIAL AND FINAL TIMES
	double ti=0.0,tf=1.0;
	Array<int> list(0);
	ControlLinear *control;
	controlSet.getControlList("ControlLinear",list);
	if(list.getSize()>0) {
		control = (ControlLinear*)controlSet.get(list[0]);
		if(control->getNumParameters()>0) {
			ti = control->getFirstTime();
			tf = control->getLastTime();
		}
	}

	// CONVERT ALL LINEAR CONTROL NODES NOT TO USE STEPS
	int i,size=list.getSize();
	for(i=0;i<size;i++) {
		control = (ControlLinear*)controlSet.get(list[i]);
		control->setUseSteps(false);
	}

	// CONSTRUCT STORAGE FILE FOR ORIGINAL
	int n=1000;
	Storage *originalStore = controlSet.constructStorage(n,ti,tf,false); 

	// PREPARE ARGUMENTS
	PropertySet properties;
	PropertyDbl *propCutoff = new PropertyDbl("cutoff_frequency",cutoff);
	PropertyDbl *propDistance = new PropertyDbl("distance",distance);
	properties.append(propCutoff);
	properties.append(propDistance);

	// SIMPLIFY
	controlSet.simplify(properties);

	// CONSTRUCT STORAGE FILE FOR SIMPLE
	Storage *simpleStore = controlSet.constructStorage(n,ti,tf,false); 
	originalStore->print("controls_simple.sto");

	// WRITE CONTROL SET TO FILE
	controlSet.print("controls_simple.ctr");
	originalStore->print("controls_original.sto");
	simpleStore->print("controls_simple.sto");

	return(0);
}
