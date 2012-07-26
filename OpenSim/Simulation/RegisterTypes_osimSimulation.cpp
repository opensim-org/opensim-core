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
#include "Model/ExternalForce.h"
#include "Model/PointToPointSpring.h"
#include "Model/BushingForce.h"
#include "Model/FunctionBasedBushingForce.h"
#include "Model/ExternalLoads.h"
#include "Model/PathActuator.h"
#include "Model/ProbeSet.h"
#include "Model/ActuatorPowerProbe.h"
#include "Model/ActuatorForceProbe.h"
#include "Model/JointPowerProbe.h"
#include "Model/SystemEnergyProbe.h"
#include "Model/MetabolicMuscle.h"
#include "Model/MetabolicMuscleSet.h"
#include "Model/MuscleMetabolicPowerProbeUmberger2003.h"
#include "Model/MuscleMetabolicPowerProbeBhargava2004.h"

#include "Control/ControlSet.h"
#include "Control/ControlSetController.h"
#include "Control/ControlConstant.h"
#include "Control/ControlLinear.h"
#include "Control/PrescribedController.h"

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
#include "SimbodyEngine/ConstantDistanceConstraint.h"
#include "SimbodyEngine/CoordinateCouplerConstraint.h"
#include "SimbodyEngine/PointOnLineConstraint.h"
#include "SimbodyEngine/RollingOnSurfaceConstraint.h"

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

#include <string>
#include <iostream>
#include <exception>

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
  try {

    Object::registerType( AnalysisSet() );
    Object::registerType( Model() );
    Object::registerType( BodyScale() );
    Object::registerType( BodyScaleSet() );
    Object::registerType( BodySet() );
    Object::registerType( ConstraintSet() );
    Object::registerType( CoordinateSet() );
    Object::registerType( ForceSet() );
    Object::registerType( ExternalLoads() );

    Object::registerType( JointSet() );
    Object::registerType( Marker() );
    Object::registerType( MarkerSet() );
    Object::registerType( PathPoint() );
    Object::registerType( PathPointSet() );
    Object::registerType( ConditionalPathPoint() );
    Object::registerType( MovingPathPoint() );
    Object::registerType( GeometryPath() );

    Object::registerType( ControlSet() );
    Object::registerType( ControlConstant() );
    Object::registerType( ControlLinear() );
    Object::registerType( ControlLinearNode() );

    Object::registerType( PathWrap() );
    Object::registerType( PathWrapSet() );
    Object::registerType( WrapCylinder() );
    Object::registerType( WrapEllipsoid() );
    Object::registerType( WrapSphere() );
    Object::registerType( WrapTorus() );
    Object::registerType( WrapObjectSet() );
    Object::registerType( WrapCylinderObst() );
    Object::registerType( WrapSphereObst() );
    Object::registerType( WrapDoubleCylinderObst() );

    // CURRENT RELEASE
    Object::registerType( SimbodyEngine() );
    Object::registerType( OpenSim::Body() );
    Object::registerType( WeldConstraint() );
    Object::registerType( PointConstraint() );
    Object::registerType( ConstantDistanceConstraint() );
    Object::registerType( CoordinateCouplerConstraint() );
    Object::registerType( CustomJoint() );
    Object::registerType( WeldJoint() );
    Object::registerType( EllipsoidJoint() );
    Object::registerType( FreeJoint() );
    Object::registerType( BallJoint() );
    Object::registerType( PinJoint() );
    Object::registerType( SliderJoint() );
    Object::registerType( TransformAxis() );
    Object::registerType( Coordinate() );
    Object::registerType( SpatialTransform() );

    Object::registerType( ContactGeometrySet() );
    Object::registerType( ContactHalfSpace() );
    Object::registerType( ContactMesh() );
    Object::registerType( ContactSphere() );
    Object::registerType( HuntCrossleyForce() );
    Object::registerType( ElasticFoundationForce() );
    Object::registerType( HuntCrossleyForce::ContactParameters() );
    Object::registerType( HuntCrossleyForce::ContactParametersSet() );
    Object::registerType( ElasticFoundationForce::ContactParameters() );
    Object::registerType( ElasticFoundationForce::ContactParametersSet() );
    Object::registerType( PointOnLineConstraint() );
    Object::registerType( RollingOnSurfaceConstraint() );
    Object::registerType( Ligament() );
    Object::registerType( PrescribedForce() );
    Object::registerType( ExternalForce() );
    Object::registerType( PointToPointSpring() );
    Object::registerType( BushingForce() );
    Object::registerType( FunctionBasedBushingForce() );

    Object::registerType( ControlSetController() );
    Object::registerType( PrescribedController() );

    Object::registerType( BushingForce() );
    Object::registerType( PathActuator() );
    Object::RegisterType( ProbeSet() );
    Object::RegisterType( ActuatorPowerProbe() );
    Object::RegisterType( ActuatorForceProbe() );
    Object::RegisterType( JointPowerProbe() );
    Object::RegisterType( SystemEnergyProbe() );
    Object::RegisterType( MetabolicMuscle() );
    Object::RegisterType( MetabolicMuscleSet() );
    Object::RegisterType( MuscleMetabolicPowerProbeUmberger2003() );
    Object::RegisterType( MuscleMetabolicPowerProbeBhargava2004() );

    // OLD Versions
    // Associate an instance with old name to help deserialization.
    // This has to be done after the new Type is registered.
    Object::renameType("ActuatorSet",       "ForceSet");
    Object::renameType("MuscleWrap",        "PathWrap");
    Object::renameType("MuscleWrapSet",     "PathWrapSet");
    Object::renameType("MusclePoint",       "PathPoint");
    Object::renameType("MuscleViaPoint",    "ConditionalPathPoint");
    Object::renameType("MovingMusclePoint", "MovingPathPoint");
    Object::renameType("MusclePointSet",    "PathPointSet");

  } catch (const std::exception& e) {
    std::cerr 
        << "ERROR during osimSimulation Object registration:\n"
        << e.what() << "\n";
  }
}


osimSimulationInstantiator::osimSimulationInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void osimSimulationInstantiator::registerDllClasses() 
{ 
        RegisterTypes_osimSimulation(); 
} 
    
