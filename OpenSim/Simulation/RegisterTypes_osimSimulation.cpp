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

#include "Model/AnalysisSet.h"
#include "Model/Model.h"
#include "Model/ForceSet.h"
#include "Model/BodyScale.h"
#include "Model/BodyScaleSet.h"
#include "Model/BodySet.h"
#include "Model/ConstraintSet.h"
#include "Model/ContactGeometry.h"
#include "Model/ContactGeometrySet.h"
#include "Model/ContactHalfSpace.h"
#include "Model/ContactMesh.h"
#include "Model/ContactSphere.h"
#include "Model/CoordinateSet.h"
#include "Model/ElasticFoundationForce.h"
#include "Model/HuntCrossleyForce.h"
#include "Model/Ligament.h"
#include "Model/JointSet.h"
#include "Model/Marker.h"
#include "Model/MarkerSet.h"
#include "Model/PathPoint.h"
#include "Model/PathPointSet.h"
#include "Model/ConditionalPathPoint.h"
#include "Model/MovingPathPoint.h"
#include "Model/GeometryPath.h"
#include "Model/PrescribedForce.h"

#include "Control/ControlSet.h"
#include "Control/ControlSetController.h"
#include "Control/ControlConstant.h"
#include "Control/ControlLinear.h"
#include "Wrap/PathWrap.h"
#include "Wrap/PathWrapSet.h"
#include "Wrap/WrapCylinder.h"
#include "Wrap/WrapEllipsoid.h"
#include "Wrap/WrapSphere.h"
#include "Wrap/WrapTorus.h"
#include "Wrap/WrapObjectSet.h"
#include "Wrap/WrapCylinderObst.h"
#include "Wrap/WrapSphereObst.h"
#include "Wrap/WrapDoubleCylinderObst.h"

#include "SimbodyEngine/SimbodyEngine.h"
#include "SimbodyEngine/Body.h"
#include "SimbodyEngine/Constraint.h"
#include "SimbodyEngine/WeldConstraint.h"
#include "SimbodyEngine/PointConstraint.h"
#include "SimbodyEngine/CoordinateCouplerConstraint.h"
#include "SimbodyEngine/PointOnLineConstraint.h"
#include "SimbodyEngine/EllipsoidJoint.h"
#include "SimbodyEngine/BallJoint.h"
#include "SimbodyEngine/PinJoint.h"
#include "SimbodyEngine/SliderJoint.h"
#include "SimbodyEngine/FreeJoint.h"
#include "SimbodyEngine/CustomJoint.h"
#include "SimbodyEngine/WeldJoint.h"
#include "SimbodyEngine/TransformAxis.h"
#include "SimbodyEngine/Coordinate.h"
#include "SimbodyEngine/SpatialTransform.h"


using namespace std;
using namespace OpenSim;

static osimSimulationInstantiator instantiator; 

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the osimSimulation library.
 */
OSIMSIMULATION_API void RegisterTypes_osimSimulation()
{
	//cout<<"RegisterTypes_osimSimulation\n";

	Object::RegisterType( AnalysisSet() );
	Object::RegisterType( Model() );
	Object::RegisterType( BodyScale() );
	Object::RegisterType( BodyScaleSet() );
	Object::RegisterType( BodySet() );
	//Object::RegisterType( BoneSet() );
	Object::RegisterType( ConstraintSet() );
	Object::RegisterType( CoordinateSet() );
	Object::RegisterType( ForceSet() );

	Object::RegisterType( JointSet() );
	Object::RegisterType( Marker() );
	Object::RegisterType( MarkerSet() );
	Object::RegisterType( PathPoint() );
	Object::RegisterType( PathPointSet() );
	Object::RegisterType( ConditionalPathPoint() );
	Object::RegisterType( MovingPathPoint() );
	Object::RegisterType( GeometryPath() );

	Object::RegisterType( ControlSet() );
	Object::RegisterType( ControlConstant() );
	Object::RegisterType( ControlLinear() );
	Object::RegisterType( ControlLinearNode() );

	Object::RegisterType( PathWrap() );
	Object::RegisterType( PathWrapSet() );
	Object::RegisterType( WrapCylinder() );
	Object::RegisterType( WrapEllipsoid() );
	Object::RegisterType( WrapSphere() );
	Object::RegisterType( WrapTorus() );
	Object::RegisterType( WrapObjectSet() );
	Object::RegisterType( WrapCylinderObst() );
	Object::RegisterType( WrapSphereObst() );
	Object::RegisterType( WrapDoubleCylinderObst() );

	//simbodyEngine
	// CURRENT RELEASE
	Object::RegisterType( SimbodyEngine() );
	Object::RegisterType( OpenSim::Body() );
	Object::RegisterType( Constraint() );
	Object::RegisterType( WeldConstraint() );
	Object::RegisterType( PointConstraint() );
	Object::RegisterType( CoordinateCouplerConstraint() );
	Object::RegisterType( CustomJoint() );
	Object::RegisterType( WeldJoint() );
	Object::RegisterType( EllipsoidJoint() );
	Object::RegisterType( FreeJoint() );
	Object::RegisterType( BallJoint() );
	Object::RegisterType( PinJoint() );
	Object::RegisterType( SliderJoint() );
	Object::RegisterType( TransformAxis() );
	Object::RegisterType( Coordinate() );
	Object::RegisterType( SpatialTransform() );

	Object::RegisterType( ContactGeometrySet() );
	Object::RegisterType( ContactHalfSpace() );
	Object::RegisterType( ContactMesh() );
	Object::RegisterType( ContactSphere() );
	Object::RegisterType( HuntCrossleyForce() );
	Object::RegisterType( ElasticFoundationForce() );
	Object::RegisterType( HuntCrossleyForce::ContactParameters() );
	Object::RegisterType( HuntCrossleyForce::ContactParametersSet() );
	Object::RegisterType( ElasticFoundationForce::ContactParameters() );
	Object::RegisterType( ElasticFoundationForce::ContactParametersSet() );
	Object::RegisterType( PointOnLineConstraint() );
	Object::RegisterType( Ligament() );
	Object::RegisterType( PrescribedForce() );

    Object::RegisterType( ControlSetController() );


	// OLD Versions
	//Associate an instance with old name to help deserialization
	// This has to be done after the new Type is registered
	Object::RenameType("ActuatorSet", ForceSet());
	Object::RenameType("MuscleWrap", PathWrap());
	Object::RenameType("MuscleWrapSet", PathWrapSet());
	Object::RenameType("MusclePoint", PathPoint());
	Object::RenameType("MuscleViaPoint", ConditionalPathPoint());
	Object::RenameType("MovingMusclePoint", MovingPathPoint());
	Object::RenameType("MusclePointSet", PathPointSet());
}


osimSimulationInstantiator::osimSimulationInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void osimSimulationInstantiator::registerDllClasses() 
{ 
        RegisterTypes_osimSimulation(); 
} 
    
