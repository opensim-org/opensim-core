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
#include <OpenSim/version.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/VisibleProperties.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/LinearSetPoint.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>

#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Analysis.h>
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
	//TODO: put these options on the command line
	LoadOpenSimLibrary("osimSimbodyEngine");

	// SET OUTPUT FORMATTING
	IO::SetDigitsPad(4);

	// REGISTER TYPES
	Object::RegisterType(VisibleObject());
	Object::RegisterType(ScaleTool());
	ScaleTool::registerTypes();

	std::string setupFilePath;
	ScaleTool* subject;
	Model* model;

   if (argc != 3)
   {
	   cout << "Not enough arguments passed to testScale" << endl;
      exit(1);
   }
	try {
		// Construct model and read parameters file
		subject = new ScaleTool(argv[2]);

		// Keep track of the folder containing setup file, wil be used to locate results to comapre against
		setupFilePath=subject->getPathToSubject();

		model = subject->createModel();

		if(!model) throw Exception("scale: ERROR- No model specified.",__FILE__,__LINE__);

		if (!subject->isDefaultModelScaler() && subject->getModelScaler().getApply())
		{
			ModelScaler& scaler = subject->getModelScaler();
			if(!scaler.processModel(model, subject->getPathToSubject(), subject->getSubjectMass())) return 1;
		}
		else
		{
			cout << "Scaling parameters disabled (apply is false) or not set. Model is not scaled." << endl;
		}

		if (!subject->isDefaultMarkerPlacer() && subject->getMarkerPlacer().getApply())
		{
			MarkerPlacer& placer = subject->getMarkerPlacer();
			if(!placer.processModel(model, subject->getPathToSubject())) return 1;
		}
		else
		{
			cout << "Marker placement parameters disabled (apply is false) or not set. No markers have been moved." << endl;
			return 1;
		}

	}
	catch(Exception &x) {
		x.print(cout);
		return 1;
	}

	Model* stdModel= new Model(setupFilePath+"subject01_simbody.osim");
	stdModel->setup();

	// Check models are equal
	if (!(*model==*stdModel))
		return 1;
	// Compare ScaleSet
	ScaleSet stdScaleSet = ScaleSet(setupFilePath+"subject01_Scale_ScaleSet.xml");
	ScaleSet& computedScaleSet = subject->getModelScaler().getScaleSet();

	if (!(computedScaleSet==stdScaleSet))
		return 1;

	cout << "Path used = " << getenv("PATH") << endl;

	delete model;
	delete subject;

	return (0);
}

