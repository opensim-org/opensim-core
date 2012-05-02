// RegisterTypes_osimTools.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimTools.h"

#include "ScaleTool.h"
//#include "IKTool.h"
//#include "FIKSTool.h"
#include "CMCTool.h"
#include "RRATool.h"
#include "ForwardTool.h"
//#include "PerturbationTool.h"
#include "AnalyzeTool.h"
#include "InverseKinematicsTool.h"
#include "InverseDynamicsTool.h"

#include "GenericModelMaker.h"
#include "IKCoordinateTask.h"
#include "IKMarkerTask.h"
#include "IKTaskSet.h"
#include "MarkerPair.h"
#include "MarkerPairSet.h"
#include "MarkerPlacer.h"
#include "Measurement.h"
#include "MeasurementSet.h"
#include "ModelScaler.h"
#include "CMC.h"
#include "CMC_Point.h"
#include "CMC_Joint.h"
#include "SMC_Joint.h"
#include "CMC_TaskSet.h"
#include "CorrectionController.h"
#include "MuscleStateTrackingTask.h"

#include <string>
#include <iostream>
#include <exception>

using namespace std;
using namespace OpenSim;

static osimToolsInstantiator instantiator;

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the osimTools library.
 */
OSIMTOOLS_API void RegisterTypes_osimTools()
{
  try {

	Object::registerType( ScaleTool() );
	//Object::registerType( IKTool() );
	Object::registerType( CMCTool() );
	Object::registerType( RRATool() );
	Object::registerType( ForwardTool() );
	Object::registerType( AnalyzeTool() );

	Object::registerType( GenericModelMaker() );
	Object::registerType( IKCoordinateTask() );
	Object::registerType( IKMarkerTask() );
	Object::registerType( IKTaskSet() );
	//Object::registerType( IKTrial() );
	//Object::registerType( IKTrialSet() );
	Object::registerType( MarkerPair() );
	Object::registerType( MarkerPairSet() );
	Object::registerType( MarkerPlacer() );
	Object::registerType( Measurement() );
	Object::registerType( MeasurementSet() );
	Object::registerType( ModelScaler() );

	Object::registerType( CorrectionController() );
	Object::registerType( CMC() );
	Object::registerType( CMC_Joint() );
	Object::registerType( CMC_Point() );
	Object::registerType( MuscleStateTrackingTask() );
	Object::registerType( CMC_TaskSet() );

	Object::registerType( SMC_Joint() );
	Object::registerType( InverseKinematicsTool() );
	Object::registerType( InverseDynamicsTool() );
	// Old versions
	Object::RenameType("rdCMC_Joint",   "CMC_Joint");
	Object::RenameType("rdCMC_Point",   "CMC_Point");
	Object::RenameType("rdCMC_TaskSet", "CMC_TaskSet");

	Object::RenameType("IKTool", "InverseKinematicsTool");

  } catch (const std::exception& e) {
    std::cerr 
        << "ERROR during osimTools Object registration:\n"
        << e.what() << "\n";
  }
}


osimToolsInstantiator::osimToolsInstantiator()
{
       registerDllClasses();
}

void osimToolsInstantiator::registerDllClasses()
{
       RegisterTypes_osimTools();
}