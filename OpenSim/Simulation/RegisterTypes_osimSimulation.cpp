/* -------------------------------------------------------------------------- *
 *                 OpenSim:  RegisterTypes_osimSimulation.cpp                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
    Object::registerType( ModelDisplayHints());
    Object::registerType( Model() );
    Object::registerType( BodyScale() );
    Object::registerType( BodyScaleSet());
    Object::registerType( FrameSet());
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
    Object::registerType( CurveProperties());
    Object::registerType( Appearance());
    Object::registerType( ModelVisualPreferences());

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

    Object::registerType( WeldJoint());
    Object::registerType( CustomJoint());
    Object::registerType( EllipsoidJoint() );
    Object::registerType( FreeJoint() );
    Object::registerType( BallJoint() );
    Object::registerType( GimbalJoint() );
    Object::registerType( UniversalJoint() );
    Object::registerType( PinJoint() );
    Object::registerType( SliderJoint() );
    Object::registerType( PlanarJoint() );
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
    using OpenSim::HuntCrossleyForce;
    using OpenSim::ElasticFoundationForce;
    Object::registerType( HuntCrossleyForce() );
    Object::registerType( ElasticFoundationForce() );
    Object::registerType( HuntCrossleyForce::ContactParameters() );
    Object::registerType( HuntCrossleyForce::ContactParametersSet() );
    Object::registerType( ElasticFoundationForce::ContactParameters() );
    Object::registerType( ElasticFoundationForce::ContactParametersSet() );

    Object::registerType( Ligament() );
    Object::registerType( PrescribedForce() );
    Object::registerType( ExternalForce() );
    Object::registerType( PointToPointSpring() );
    Object::registerType( ExpressionBasedPointToPointForce() );
    Object::registerType( PathSpring() );
    Object::registerType( BushingForce() );
    Object::registerType( FunctionBasedBushingForce() );
    Object::registerType( ExpressionBasedBushingForce() );

    Object::registerType( ControlSetController() );
    Object::registerType( PrescribedController() );
    Object::registerType( ToyReflexController() );

    Object::registerType( PathActuator() );
    Object::registerType( ProbeSet() );
    Object::registerType( JointInternalPowerProbe() );
    Object::registerType( SystemEnergyProbe() );
    Object::registerType( Umberger2010MuscleMetabolicsProbe() );
    Object::registerType( Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet() );
    Object::registerType( Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter() );
    Object::registerType( Bhargava2004MuscleMetabolicsProbe() );
    Object::registerType( Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet() );
    Object::registerType( Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter() );

    Object::registerType( StatesTrajectoryReporter() );

    // Register commonly used Connectors for de/serialization
    Object::registerType(Connector<Frame>());
    Object::registerType(Connector<PhysicalFrame>());
    Object::registerType(OpenSim::Connector<OpenSim::Body>());

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
    
