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
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace std;
using namespace OpenSim;

string filesToCompare[] = {
	"subject_trial_ik.mot"
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
// Eran: comment out everything for now because we now use InvestigationIK
#if 0
	// Construct model and read parameters file
	Object::RegisterType(ScaleTool());
	ScaleTool::registerTypes();
	ScaleTool* subject = new ScaleTool("subject_ik_setup.xml");
	Model* model = subject->createModel();

	//----------------------- Model scaling section
	if (!subject->isDefaultModelScaler())
	{
		ModelScaler& scaler = subject->getModelScaler();
		scaler.processModel(model, subject->getPathToSubject(), subject->getMass());
	}
	else
	{
		cout << "ModelScaler parameters have not been defined. The generic model will not be scaled." << endl;
	}

	//----------------------- Marker placement section
	if (!subject->isDefaultMarkerPlacer())
	{
		MarkerPlacer& placer = subject->getMarkerPlacer();
		placer.processModel(model, subject->getPathToSubject());
	}
	else
	{
		cout << "MarkerPlacer parameters have not been defined. No markers have been moved." << endl;
	}

	//--------------------- IK proper section
	{
		try 
		{
			if (!subject->isDefaultIKSolver())
			{
				SimmIKSolver& solver = subject->getIKSolver();
				solver.processModel(model, subject->getPathToSubject());
			}
			else
			{
				cout << "IK Solver parameters not set. No IK has been performed." << endl;
			}
		}
		catch (Exception &x)
		{
			x.print(cout);
			cout << "Press Return to continue." << endl;
			cout.flush();
			int c = getc( stdin );
			return 1;
		}
	}
	delete subject;

	/* Compare results with standard*/
	bool success = true;
	for (int i=0; i < 1 && success; i++){
		string command = "cmp "+filesToCompare[i]+" "+"std_"+filesToCompare[i];
		success = success && (system(command.c_str())==0);
	}

	return (success?0:1);
#endif
}

