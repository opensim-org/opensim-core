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

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimTools.h"

#include "ScaleTool.h"
#include "IKTool.h"
//#include "FIKSTool.h"
#include "CMCTool.h"
#include "ForwardTool.h"
//#include "PerturbationTool.h"
#include "AnalyzeTool.h"

#include "GenericModelMaker.h"
#include "IKCoordinateTask.h"
#include "IKMarkerTask.h"
#include "IKTaskSet.h"
#include "IKTrial.h"
#include "IKTrialSet.h"
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
	//cout<<"RegisterTypes_osimTools\n";

	Object::RegisterType( ScaleTool() );
	Object::RegisterType( IKTool() );
	//Object::RegisterType( FIKSTool() );
	Object::RegisterType( CMCTool() );
	Object::RegisterType( ForwardTool() );
	//Object::RegisterType( PerturbationTool() );
	Object::RegisterType( AnalyzeTool() );

	Object::RegisterType( GenericModelMaker() );
	Object::RegisterType( IKCoordinateTask() );
	Object::RegisterType( IKMarkerTask() );
	Object::RegisterType( IKTaskSet() );
	Object::RegisterType( IKTrial() );
	Object::RegisterType( IKTrialSet() );
	Object::RegisterType( MarkerPair() );
	Object::RegisterType( MarkerPairSet() );
	Object::RegisterType( MarkerPlacer() );
	Object::RegisterType( Measurement() );
	Object::RegisterType( MeasurementSet() );
	Object::RegisterType( ModelScaler() );

	Object::RegisterType( CorrectionController() );
	Object::RegisterType( CMC() );
	Object::RegisterType( CMC_Joint() );
	Object::RegisterType( CMC_Point() );
	Object::RegisterType( CMC_TaskSet() );

	Object::RegisterType( SMC_Joint() );
	// Old versions
	Object::RenameType("rdCMC_Joint", CMC_Joint());
	Object::RenameType("rdCMC_Point", CMC_Point());
	Object::RenameType("rdCMC_TaskSet", CMC_TaskSet());


}


osimToolsInstantiator::osimToolsInstantiator()
{
       registerDllClasses();
}

void osimToolsInstantiator::registerDllClasses()
{
       RegisterTypes_osimTools();
}
