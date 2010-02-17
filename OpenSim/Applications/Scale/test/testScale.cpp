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
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Tools/IKSolverImpl.h>
#include <OpenSim/Tools/IKTarget.h>




using namespace OpenSim;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw(exception());}

#define ASSERT_EQUAL(expected, found, tolerance) {double tol = std::max((tolerance), std::abs((expected)*(tolerance))); if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}


bool scaleGait2354()
{
	// SET OUTPUT FORMATTING
	IO::SetDigitsPad(4);

	std::string setupFilePath;
	ScaleTool* subject;
	Model* model;

	// Remove old results if any
	FILE* file2Remove = IO::OpenFile(setupFilePath+"subject01_scaleSet_applied.xml", "w");
	fclose(file2Remove);
	file2Remove = IO::OpenFile(setupFilePath+"subject01_simbody.osim", "w");
	fclose(file2Remove);

	// Construct model and read parameters file
	subject = new ScaleTool("subject01_Setup_Scale.xml");

	// Keep track of the folder containing setup file, wil be used to locate results to comapre against
	setupFilePath=subject->getPathToSubject();

	model = subject->createModel();

    SimTK::State& s = model->getSystem().updDefaultState();
    model->getSystem().realize(s, SimTK::Stage::Position );


	if(!model) {
//       throw Exception("scale: ERROR- No model specified.",__FILE__,__LINE__);
        printf("scale: ERROR- No model specified.");
    }


	if (!subject->isDefaultModelScaler() && subject->getModelScaler().getApply()) {
		ModelScaler& scaler = subject->getModelScaler();
	   if( false == scaler.processModel(s, model, subject->getPathToSubject(), subject->getSubjectMass())) {
           return(false);
       }
	}
	else {
        return(false);
	}

/* Since we compare only scale factors the following block is unsed. Need to update when ScaleTool
	really works and performs marker placement.

	if (!subject->isDefaultMarkerPlacer() && subject->getMarkerPlacer().getApply()) {
		MarkerPlacer& placer = subject->getMarkerPlacer();
	    if( false == placer.processModel(s, model, subject->getPathToSubject())) return(false);
	}
	else {
        return(1);
	}
	*/
	// Compare ScaleSet
	ScaleSet stdScaleSet = ScaleSet(setupFilePath+"std_subject01_scaleSet_applied.xml");

	const ScaleSet& computedScaleSet = ScaleSet(setupFilePath+"subject01_scaleSet_applied.xml");

	if(!(computedScaleSet == stdScaleSet)) 
		return(false);
    

	delete model;
	delete subject;

    return(true);
}


int main()
{
    try {
 
		if( false == scaleGait2354() ) {
            std::cout << "FAILED" << std::endl;
            return(1);
        }

    }
    catch(const std::exception& e) {
        cout << "exception: " << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

