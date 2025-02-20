/* -------------------------------------------------------------------------- *
 *                 OpenSim:  RegisterTypes_osimSimulation.cpp                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimSimulation.h"

#include "Model/Model.h"

#include "Model/Frame.h"
#include "Model/PhysicalFrame.h"
#include "Model/Ground.h"
#include "Model/OffsetFrame.h"
#include "Model/PhysicalOffsetFrame.h"
#include "Model/StationDefinedFrame.h"

#include "Model/AnalysisSet.h"
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
#include "Model/CoordinateLimitForce.h"
#include "Model/CoordinateSet.h"
#include "Model/ElasticFoundationForce.h"
#include "Model/HuntCrossleyForce.h"
#include "Model/SmoothSphereHalfSpaceForce.h"
#include "Model/Ligament.h"
#include "Model/Blankevoort1991Ligament.h"
#include "Model/JointSet.h"
#include "Model/Marker.h"
#include "Model/Station.h"
#include "Model/MarkerSet.h"
#include "Model/PathPoint.h"
#include "Model/PathPointSet.h"
#include "Model/ConditionalPathPoint.h"
#include "Model/MovingPathPoint.h"
#include "Model/GeometryPath.h"
#include "Model/FunctionBasedPath.h"
#include "Model/PrescribedForce.h"
#include "Model/ExternalForce.h"
#include "Model/PointToPointSpring.h"
#include "Model/ExpressionBasedPointToPointForce.h"
#include "Model/PathSpring.h"
#include "Model/BushingForce.h"
#include "Model/FunctionBasedBushingForce.h"
#include "Model/ExpressionBasedBushingForce.h"
#include "Model/ExternalLoads.h"
#include "Model/PathActuator.h"
#include "Model/ProbeSet.h"
#include "Model/ActuatorPowerProbe.h"
#include "Model/ActuatorForceProbe.h"
#include "Model/JointInternalPowerProbe.h"
#include "Model/SystemEnergyProbe.h"
#include "Model/Umberger2010MuscleMetabolicsProbe.h"
#include "Model/Bhargava2004MuscleMetabolicsProbe.h"
#include "Model/Bhargava2004SmoothedMuscleMetabolics.h"
#include "Model/Appearance.h"
#include "Model/Geometry.h"

#include "Model/ModelVisualPreferences.h"
#include "Model/ExpressionBasedCoordinateForce.h"

#include "Control/ControlSet.h"
#include "Control/ControlSetController.h"
#include "Control/ControlConstant.h"
#include "Control/ControlLinear.h"
#include "Control/PrescribedController.h"
#include "Control/InputController.h"
#include "Control/SynergyController.h"

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
#include "SimbodyEngine/GimbalJoint.h"
#include "SimbodyEngine/UniversalJoint.h"
#include "SimbodyEngine/PinJoint.h"
#include "SimbodyEngine/SliderJoint.h"
#include "SimbodyEngine/PlanarJoint.h"
#include "SimbodyEngine/FreeJoint.h"
#include "SimbodyEngine/CustomJoint.h"
#include "SimbodyEngine/WeldJoint.h"
#include "SimbodyEngine/ScapulothoracicJoint.h"
#include "OpenSim/Simulation/SimbodyEngine/ConstantCurvatureJoint.h"
#include "SimbodyEngine/TransformAxis.h"
#include "SimbodyEngine/Coordinate.h"
#include "SimbodyEngine/SpatialTransform.h"
#include "OpenSense/IMUPlacer.h"
#include "OpenSense/IMU.h"
#include "OrientationsReference.h"
#include "StatesTrajectoryReporter.h"
#include "TableProcessor.h"
#include "MarkersReference.h"
#include "PositionMotion.h"
#include "Control/InputController.h"

#include <string>
#include <iostream>
#include <exception>

using namespace std;
using namespace OpenSim;

#ifndef OPENSIM_DISABLE_STATIC_TYPE_REGISTRATION
    static osimSimulationInstantiator instantiator;
#endif

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
    Object::registerType( BodyScaleSet());
    Object::registerType( BodySet());
    Object::registerType( ComponentSet() );
    Object::registerType( ControllerSet() );
    Object::registerType( ConstraintSet() );
    Object::registerType( CoordinateSet() );
    Object::registerType( ForceSet() );
    Object::registerType( ExternalLoads() );

    Object::registerType( JointSet() );
    Object::registerType( Marker() );
    Object::registerType( Station() );
    Object::registerType( MarkerSet() );
    Object::registerType( PathPoint() );
    Object::registerType( PathPointSet() );
    Object::registerType( ConditionalPathPoint() );
    Object::registerType( MovingPathPoint() );
    Object::registerType( SurfaceProperties());
    Object::registerType( Appearance());
    Object::registerType( ModelVisualPreferences());

    Object::registerType(MarkersReference());
    Object::registerType(MarkerWeight());
    Object::registerType(Set<MarkerWeight>());


    Object::registerType( Brick());
    Object::registerType( Sphere());
    Object::registerType( Cylinder());
    Object::registerType( Ellipsoid());
    Object::registerType( Mesh());
    Object::registerType( Torus());
    Object::registerType( Cone());
    Object::registerType( LineGeometry());
    Object::registerType( FrameGeometry());
    Object::registerType( Arrow());
    Object::registerType( GeometryPath());
    Object::registerType( FunctionBasedPath());

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
    Object::registerType( OpenSim::Ground());
    Object::registerType( PhysicalOffsetFrame());
    Object::registerType( StationDefinedFrame());

    Object::registerType( WeldJoint());
    Object::registerType( CustomJoint());
    Object::registerType( EllipsoidJoint() );
    Object::registerType( FreeJoint() );
    Object::registerType( BallJoint() );
    Object::registerType( GimbalJoint() );
    Object::registerType( ScapulothoracicJoint() );
    Object::registerType( UniversalJoint() );
    Object::registerType( PinJoint() );
    Object::registerType( SliderJoint() );
    Object::registerType( PlanarJoint() );
    Object::registerType( ConstantCurvatureJoint() );
    Object::registerType( TransformAxis() );
    Object::registerType( Coordinate() );
    Object::registerType( SpatialTransform() );

    Object::registerType(WeldConstraint());
    Object::registerType(PointConstraint());
    Object::registerType(ConstantDistanceConstraint());
    Object::registerType(CoordinateCouplerConstraint());
    Object::registerType(PointOnLineConstraint());
    Object::registerType(RollingOnSurfaceConstraint());

    Object::registerType( ContactGeometrySet() );
    Object::registerType( ContactHalfSpace() );
    Object::registerType( ContactMesh() );
    Object::registerType( ContactSphere() );
    Object::registerType( CoordinateLimitForce() );
    Object::registerType( SmoothSphereHalfSpaceForce() );
    Object::registerType( HuntCrossleyForce() );
    Object::registerType( ElasticFoundationForce() );
    Object::registerType( HuntCrossleyForce::ContactParameters() );
    Object::registerType( HuntCrossleyForce::ContactParametersSet() );
    Object::registerType( ElasticFoundationForce::ContactParameters() );
    Object::registerType( ElasticFoundationForce::ContactParametersSet() );

    Object::registerType( Ligament() );
    Object::registerType( Blankevoort1991Ligament() );
    Object::registerType( PrescribedForce() );
    Object::registerType( ExternalForce() );
    Object::registerType( PointToPointSpring() );
    Object::registerType( ExpressionBasedPointToPointForce() );
    Object::registerType( PathSpring() );
    Object::registerType( BushingForce() );
    Object::registerType( FunctionBasedBushingForce() );
    Object::registerType( ExpressionBasedBushingForce() );
    Object::registerType( ExpressionBasedCoordinateForce() );

    Object::registerType( ControlSetController() );
    Object::registerType( PrescribedController() );
    Object::registerType( SynergyVector() );
    Object::registerType( SynergyController() );

    Object::registerType( PathActuator() );
    Object::registerType( ProbeSet() );
    Object::registerType( JointInternalPowerProbe() );
    Object::registerType( SystemEnergyProbe() );
    Object::registerType( ActuatorForceProbe() );
    Object::registerType( ActuatorPowerProbe() );
    Object::registerType( Umberger2010MuscleMetabolicsProbe() );
    Object::registerType( Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet() );
    Object::registerType( Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter() );
    Object::registerType( Bhargava2004MuscleMetabolicsProbe() );
    Object::registerType( Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet() );
    Object::registerType( Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter() );
    Object::registerType( Bhargava2004SmoothedMuscleMetabolics());
    Object::registerType( Bhargava2004SmoothedMuscleMetabolics_MuscleParameters() );
    Object::registerType( OrientationWeight() );

    Object::registerType( IMUPlacer() );
    Object::registerType( IMU() );
    Object::registerType( StatesTrajectoryReporter() );

    Object::registerType( TableProcessor() );

    Object::registerType( TabOpLowPassFilter() );
    Object::registerType( TabOpUseAbsoluteStateNames() );
    Object::registerType( TabOpAppendCoupledCoordinateValues() );
    Object::registerType( PositionMotion() );

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

    Object::renameType("MuscleMetabolicPowerProbeUmberger2010",  
        "Umberger2010MuscleMetabolicsProbe");

    Object::renameType("MuscleMetabolicPowerProbeUmberger2010_MetabolicMuscleParameter",  
        "Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter");

    Object::renameType("MuscleMetabolicPowerProbeUmberger2010_MetabolicMuscleParameterSet",  
        "Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet");

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
    
