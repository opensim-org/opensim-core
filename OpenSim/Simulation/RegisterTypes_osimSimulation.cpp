// RegisterTypes_osimSimulation.cpp
// author: Frank C. Anderson
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

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimSimulation.h"

#include "Model/ContactForceSet.h"
#include "Model/Force.h"
#include "Model/AnalysisSet.h"
#include "Model/Model.h"
#include "Model/ActuatorSet.h"
#include "Model/BodyScale.h"
#include "Model/BodyScaleSet.h"
#include "Model/BodySet.h"
#include "Model/ConstraintSet.h"
#include "Model/CoordinateSet.h"
#include "Model/DofSet01_05.h"
#include "Model/TransformAxisSet.h"
#include "Model/GeneralizedForce.h"
#include "Model/JointSet.h"
#include "Model/Marker.h"
#include "Model/MarkerSet.h"
#include "Model/MusclePoint.h"
#include "Model/MuscleViaPoint.h"
#include "Model/MovingMusclePoint.h"
#include "Model/SpeedSet.h"
#include "Model/SpeedSet.h"
#include "Control/ControlSet.h"
#include "Control/ControlConstant.h"
#include "Control/ControlLinear.h"
#include "Wrap/MuscleWrap.h"
#include "Wrap/MuscleWrapSet.h"
#include "Wrap/WrapCylinder.h"
#include "Wrap/WrapEllipsoid.h"
#include "Wrap/WrapSphere.h"
#include "Wrap/WrapTorus.h"
#include "Wrap/WrapObjectSet.h"
#include "Wrap/WrapCylinderObst.h"
#include "Wrap/WrapSphereObst.h"
#include "Wrap/WrapDoubleCylinderObst.h"

using namespace std;
using namespace OpenSim;

#ifndef STATIC_OSIM_LIBS
static osimSimulationInstantiator instantiator; 
#endif

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the osimSimulation library.
 */
OSIMSIMULATION_API void RegisterTypes_osimSimulation()
{
	//cout<<"RegisterTypes_osimSimulation\n";

	Object::RegisterType( Force() );
	Object::RegisterType( AnalysisSet() );
	Object::RegisterType( Model() );
	Object::RegisterType( ActuatorSet() );
	Object::RegisterType( BodyScale() );
	Object::RegisterType( BodyScaleSet() );
	Object::RegisterType( BodySet() );
	//Object::RegisterType( BoneSet() );
	Object::RegisterType( ConstraintSet() );
	Object::RegisterType( ContactForceSet() );
	Object::RegisterType( CoordinateSet() );
	Object::RegisterType( DofSet01_05() );
	Object::RegisterType( TransformAxisSet() );
	Object::RegisterType( GeneralizedForce() );
	Object::RegisterType( JointSet() );
	Object::RegisterType( Marker() );
	Object::RegisterType( MarkerSet() );
	Object::RegisterType( MusclePoint() );
	Object::RegisterType( MuscleViaPoint() );
	Object::RegisterType( MovingMusclePoint() );
	Object::RegisterType( SpeedSet() );

	Object::RegisterType( ControlSet() );
	Object::RegisterType( ControlConstant() );
	Object::RegisterType( ControlLinear() );
	Object::RegisterType( ControlLinearNode() );

	Object::RegisterType( MuscleWrap() );
	Object::RegisterType( MuscleWrapSet() );
	Object::RegisterType( WrapCylinder() );
	Object::RegisterType( WrapEllipsoid() );
	Object::RegisterType( WrapSphere() );
	Object::RegisterType( WrapTorus() );
	Object::RegisterType( WrapObjectSet() );
	Object::RegisterType( WrapCylinderObst() );
	Object::RegisterType( WrapSphereObst() );
   Object::RegisterType( WrapDoubleCylinderObst() );
}

osimSimulationInstantiator::osimSimulationInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void osimSimulationInstantiator::registerDllClasses() 
{ 
        RegisterTypes_osimSimulation(); 
} 
    
