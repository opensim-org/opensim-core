// testIK.cpp
// Author: Ayman Habib based on Peter Loan's version
/* Copyright (c)  2005, Stanford University and Peter Loan.
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
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Common/SimmMotionData.h>
#include <OpenSim/Tools/IKSolverImpl.h>
#include <OpenSim/Tools/IKTarget.h>

using namespace std;
using namespace OpenSim;

string filesToCompare[] = {
							"CrouchGaitSP.jnt",
							"CrouchGaitSP.msl",
							"CrouchGaitSP.xml",
							"CrouchGaitScale.xml"

};
//______________________________________________________________________________
/**
 * Test program to read SIMM model elements from an XML file.
 *
 * @param argc Number of command line arguments (should be 1).
 * @param argv Command line arguments:  simmReadXML inFile
 */
int main(int argc,char **argv)
{
	// Construct model and read parameters file
	//Object::RegisterType(VisibleObject());
	Object::RegisterType(ScaleTool());
	ScaleTool::registerTypes();
	ScaleTool* subject = new ScaleTool("CrouchGait.xml");
	Model* model = subject->createModel();
	if (!subject->isDefaultModelScaler())
	{
		ModelScaler& scaler = subject->getModelScaler();
		scaler.processModel(model, subject->getPathToSubject(), subject->getSubjectMass());
	}
	else
	{
		cout << "Scaling parameters not set. Model is not scaled." << endl;
	}

	if (!subject->isDefaultMarkerPlacer())
	{
		MarkerPlacer& placer = subject->getMarkerPlacer();
		placer.processModel(model, subject->getPathToSubject());
	}
	else
	{
		cout << "Marker placement parameters not set. No markers have been moved." << endl;
	}

	delete model;
	delete subject;

	/* Compare results with standard*/
	bool success = true;
	for (int i=0; i < 4 && success; i++){
		string command = "cmp "+filesToCompare[i]+" "+"std_"+filesToCompare[i];
		success = success && (system(command.c_str())==0);
	}
	cout << "Path used = " << getenv("PATH") << endl;

	return (success?0:1);
}
	
