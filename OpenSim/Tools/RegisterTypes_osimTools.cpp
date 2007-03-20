// RegisterTypes_osimTools.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS,
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
// THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimTools.h"

#include "ScaleTool.h"
#include "IKTool.h"
#include "CMCTool.h"
#include "ForwardTool.h"
#include "PerturbationTool.h"

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

#include "rdCMC_Joint.h"
#include "rdCMC_TaskSet.h"

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
	Object::RegisterType( CMCTool() );
	Object::RegisterType( ForwardTool() );
	Object::RegisterType( PerturbationTool() );

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

	Object::RegisterType( rdCMC_Joint() );
	Object::RegisterType( rdCMC_TaskSet() );
}


osimToolsInstantiator::osimToolsInstantiator()
{
       registerDllClasses();
}

void osimToolsInstantiator::registerDllClasses()
{
       RegisterTypes_osimTools();
}
