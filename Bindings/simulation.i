
// osimSimulation
%include <OpenSim/Simulation/osimSimulationDLL.h>


%typedef SimTK::DecorativeGeometry::Representation VisualRepresentation;

%include <OpenSim/Simulation/Model/Appearance.h>
%include <OpenSim/Simulation/Model/Geometry.h>
%include <OpenSim/Simulation/Model/ModelComponent.h>
%template(SetModelComponents) OpenSim::Set<OpenSim::ModelComponent, OpenSim::ModelComponent>;
%include <OpenSim/Simulation/Model/ModelComponentSet.h>
%template(ModelComponentSetModelComponent) OpenSim::ModelComponentSet<OpenSim::ModelComponent>;
%include <OpenSim/Simulation/Model/ComponentSet.h>

%template(SetMuscles) OpenSim::Set<OpenSim::Muscle, OpenSim::Object>;

%include <OpenSim/Simulation/Solver.h>
%include <OpenSim/Simulation/InverseDynamicsSolver.h>
%include <OpenSim/Simulation/MomentArmSolver.h>

%include <OpenSim/Simulation/Model/Frame.h>
// Following three lines hacked in out of order to work around WrapObjects use
// in PhysicalFrame
%include <OpenSim/Simulation/Wrap/WrapObject.h>
%template(SetWrapObject) OpenSim::Set<OpenSim::WrapObject, OpenSim::ModelComponent>;
%template(ModelComponentSetWrapObjects) OpenSim::ModelComponentSet<OpenSim::WrapObject>;
%include <OpenSim/Simulation/Wrap/WrapObjectSet.h>

%include <OpenSim/Simulation/Model/PhysicalFrame.h>
%include <OpenSim/Simulation/Model/Ground.h>
%include <OpenSim/Simulation/Model/OffsetFrame.h>
%template(PhysicalFrameWithOffset)   OpenSim::OffsetFrame<OpenSim::PhysicalFrame>;
%include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
%template(SetFrames) OpenSim::Set<OpenSim::Frame, OpenSim::ModelComponent>;
%template(ModelComponentSetFrames)
OpenSim::ModelComponentSet<OpenSim::Frame>;
%include <OpenSim/Simulation/Model/StationDefinedFrame.h>

%include <OpenSim/Simulation/SimbodyEngine/Body.h>
%template(SetBodies) OpenSim::Set<OpenSim::Body, OpenSim::ModelComponent>;
%template(ModelComponentSetBodies)
OpenSim::ModelComponentSet<OpenSim::Body>;
%include <OpenSim/Simulation/Model/BodySet.h>

%include <OpenSim/Simulation/Model/BodyScale.h>
%template(SetBodyScales) OpenSim::Set<OpenSim::BodyScale, OpenSim::Object>;
%include <OpenSim/Simulation/Model/BodyScaleSet.h>

%include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
%warnfilter(509) OpenSim::TransformAxis;
namespace OpenSim {
%warnfilter(509) TransformAxis::setFunction;
}
%include <OpenSim/Simulation/SimbodyEngine/TransformAxis.h>
%include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
%include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
%template(SetCoordinates) OpenSim::Set<OpenSim::Coordinate, OpenSim::Object>;
%include <OpenSim/Simulation/Model/CoordinateSet.h>

%include <OpenSim/Simulation/SimbodyEngine/Joint.h>
%template(SetJoints) OpenSim::Set<OpenSim::Joint, OpenSim::ModelComponent>;
%template(ModelComponentSetJoints)
OpenSim::ModelComponentSet<OpenSim::Joint>;
%include <OpenSim/Simulation/Model/JointSet.h>

%include <OpenSim/Simulation/SimbodyEngine/Constraint.h>
%template(SetConstraints) OpenSim::Set<OpenSim::Constraint, OpenSim::ModelComponent>;
%template(ModelComponentSetConstraints)
OpenSim::ModelComponentSet<OpenSim::Constraint>;
%include <OpenSim/Simulation/Model/ConstraintSet.h>

%include <OpenSim/Simulation/Model/Force.h>
%include <OpenSim/Simulation/Model/ForceProducer.h>

%template(SetForces) OpenSim::Set<OpenSim::Force, OpenSim::ModelComponent>;
%template(ModelComponentSetForces) OpenSim::ModelComponentSet<OpenSim::Force>;
%include <OpenSim/Simulation/Model/ForceSet.h>
%include <OpenSim/Simulation/Model/ExternalForce.h>
%template(SetExternalForces) OpenSim::Set<OpenSim::ExternalForce, OpenSim::ModelComponent>;
%template(ModelComponentSetExternalForces) OpenSim::ModelComponentSet<OpenSim::ExternalForce>;

%include <OpenSim/Simulation/Model/TwoFrameLinker.h>
%template(TwoFrameLinkerForce) OpenSim::TwoFrameLinker<OpenSim::Force, OpenSim::PhysicalFrame>;
%template(TwoFrameLinkerForceProducer) OpenSim::TwoFrameLinker<OpenSim::ForceProducer, OpenSim::PhysicalFrame>;
%template(TwoFrameLinkerConstraint) OpenSim::TwoFrameLinker<OpenSim::Constraint, OpenSim::PhysicalFrame>;

%include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/EllipsoidJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/BallJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/GimbalJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/UniversalJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/ScapulothoracicJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/ConstantCurvatureJoint.h>

%include <OpenSim/Simulation/SimbodyEngine/WeldConstraint.h>
%include <OpenSim/Simulation/SimbodyEngine/PointConstraint.h>
%include <OpenSim/Simulation/SimbodyEngine/ConstantDistanceConstraint.h>
namespace OpenSim {
%warnfilter(509) CoordinateCouplerConstraint::setFunction;
}
%include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>
%include <OpenSim/Simulation/SimbodyEngine/PointOnLineConstraint.h>

%include <OpenSim/Simulation/Control/Controller.h>
%template(SetControllers) OpenSim::Set<OpenSim::Controller, OpenSim::ModelComponent>;
%template(ModelComponentSetControllers)
OpenSim::ModelComponentSet<OpenSim::Controller>;
%include <OpenSim/Simulation/Model/ControllerSet.h>

%include <OpenSim/Simulation/Model/ExternalLoads.h>
%include <OpenSim/Simulation/Model/PrescribedForce.h>
%include <OpenSim/Simulation/Model/CoordinateLimitForce.h>

%include <OpenSim/Simulation/Model/ContactGeometry.h>
%template(SetContactGeometry) OpenSim::Set<OpenSim::ContactGeometry, OpenSim::ModelComponent>;
%template(ModelComponentSetContactGeometry) OpenSim::ModelComponentSet<OpenSim::ContactGeometry>;
%include <OpenSim/Simulation/Model/ContactGeometrySet.h>
%include <OpenSim/Simulation/Model/ContactHalfSpace.h>
%include <OpenSim/Simulation/Model/ContactMesh.h>
%include <OpenSim/Simulation/Model/ElasticFoundationForce.h>
%include <OpenSim/Simulation/Model/HuntCrossleyForce.h>
%include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>

namespace OpenSim {
    %ignore Smith2018ContactMesh::OBBTreeNode;
}
%include <OpenSim/Simulation/Model/Smith2018ContactMesh.h>
%include <OpenSim/Simulation/Model/Smith2018ArticularContactForce.h>

%include <OpenSim/Simulation/Model/Actuator.h>
%template(SetActuators) OpenSim::Set<OpenSim::Actuator, OpenSim::Object>;
%include <OpenSim/Simulation/Model/Analysis.h>
%template(SetAnalysis) OpenSim::Set<OpenSim::Analysis, OpenSim::Object>;
%include <OpenSim/Simulation/Model/AnalysisSet.h>

%include <OpenSim/Simulation/Control/Control.h>
%template(SetControls) OpenSim::Set<OpenSim::Control, OpenSim::Object>;
%include <OpenSim/Simulation/Control/ControlSet.h>
%include <OpenSim/Simulation/Control/ControlConstant.h>
%include <OpenSim/Simulation/Control/ControlLinearNode.h>
%template(SetControlNodes) OpenSim::ArrayPtrs<OpenSim::ControlLinearNode>;
%include <OpenSim/Simulation/Control/ControlLinear.h>
%include <OpenSim/Simulation/Control/Controller.h>
%include <OpenSim/Simulation/Control/PrescribedController.h>
%include <OpenSim/Simulation/Control/InputController.h>
%include <OpenSim/Simulation/Control/SynergyController.h>

%include <OpenSim/Simulation/Manager/Manager.h>
%include <OpenSim/Simulation/Model/AbstractTool.h>

%include <OpenSim/Simulation/Model/Point.h>
%include <OpenSim/Simulation/Model/Station.h>
%include <OpenSim/Simulation/Model/Point.h>
%include <OpenSim/Simulation/Model/Marker.h>
%template(SetMarkers) OpenSim::Set<OpenSim::Marker, OpenSim::ModelComponent>;
%template(ModelComponentSetMarkers) OpenSim::ModelComponentSet<OpenSim::Marker>;
%include <OpenSim/Simulation/Model/MarkerSet.h>

// WrapObject is included up above.
%include <OpenSim/Simulation/Wrap/WrapSphere.h>
%include <OpenSim/Simulation/Wrap/WrapCylinder.h>
%include <OpenSim/Simulation/Wrap/WrapTorus.h>
%include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
%include <OpenSim/Simulation/Wrap/PathWrap.h>
%template(SetPathWrap) OpenSim::Set<OpenSim::PathWrap, OpenSim::Object>;
%include <OpenSim/Simulation/Wrap/PathWrapSet.h>

%include <OpenSim/Simulation/Model/Probe.h>
%template(SetProbes) OpenSim::Set<OpenSim::Probe, OpenSim::ModelComponent>;
%template(ModelComponentSetProbes) OpenSim::ModelComponentSet<OpenSim::Probe>;
%include <OpenSim/Simulation/Model/ProbeSet.h>
%include <OpenSim/Simulation/Model/SystemEnergyProbe.h>
%include <OpenSim/Simulation/Model/JointInternalPowerProbe.h>
%include <OpenSim/Simulation/Model/ActuatorPowerProbe.h>
%include <OpenSim/Simulation/Model/ActuatorForceProbe.h>
%include <OpenSim/Simulation/Model/MuscleActiveFiberPowerProbe.h>
%include <OpenSim/Simulation/Model/Bhargava2004MuscleMetabolicsProbe.h>
%include <OpenSim/Simulation/Model/Bhargava2004SmoothedMuscleMetabolics.h>
%include <OpenSim/Simulation/Model/Umberger2010MuscleMetabolicsProbe.h>
%include <OpenSim/Simulation/Model/ModelVisualPreferences.h>
%include <OpenSim/Simulation/Model/ModelVisualizer.h>
%copyctor OpenSim::Model;
%include <OpenSim/Simulation/Model/Model.h>

%include <OpenSim/Simulation/Model/AbstractPathPoint.h>
%include <OpenSim/Simulation/Model/PathPoint.h>
%include <OpenSim/Simulation/Wrap/PathWrapPoint.h>
%include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
%include <OpenSim/Simulation/Model/MovingPathPoint.h>
%template(SetOfPathPoints) OpenSim::Set<OpenSim::AbstractPathPoint>;
%template(ArrayPathPoint) OpenSim::Array<OpenSim::AbstractPathPoint*>;
%include <OpenSim/Simulation/Model/PathPointSet.h>

%include <OpenSim/Simulation/Model/PointForceDirection.h>
%template(ArrayPointForceDirection) OpenSim::Array<OpenSim::PointForceDirection*>;
%include <OpenSim/Simulation/Model/AbstractGeometryPath.h>
%include <OpenSim/Simulation/Model/GeometryPath.h>
%include <OpenSim/Simulation/Model/FunctionBasedPath.h>
%include <OpenSim/Simulation/Model/Scholz2015GeometryPath.h>
%include <OpenSim/Simulation/Model/Ligament.h>
%include <OpenSim/Simulation/Model/Blankevoort1991Ligament.h>
%include <OpenSim/Simulation/Model/PathActuator.h>
%include <OpenSim/Simulation/Model/Muscle.h>
%include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>
%include <OpenSim/Simulation/Model/PointToPointSpring.h>
%include <OpenSim/Simulation/Model/ExpressionBasedPointToPointForce.h>
%include <OpenSim/Simulation/Model/ExpressionBasedCoordinateForce.h>

%include <OpenSim/Simulation/Model/PathSpring.h>
%include <OpenSim/Simulation/Model/ExpressionBasedPathForce.h>
%include <OpenSim/Simulation/Model/BushingForce.h>
%include <OpenSim/Simulation/Model/FunctionBasedBushingForce.h>
%include <OpenSim/Simulation/Model/ExpressionBasedBushingForce.h>

%include <OpenSim/Simulation/Solver.h>

%include <OpenSim/Simulation/Reference.h>
%template(ReferenceVec3) OpenSim::Reference_<SimTK::Vec3>;
%template(ReferenceDouble) OpenSim::Reference_<double>;
%template(ReferenceRotation) OpenSim::Reference_<SimTK::Rotation_<double>>;
%template(StreamableReferenceRotation) OpenSim::StreamableReference_<SimTK::Rotation_<double>>;

%template(SimTKArrayCoordinateReference) SimTK::Array_<OpenSim::CoordinateReference>;


%shared_ptr(ReferenceVec3);
%shared_ptr(ReferenceDouble);
%shared_ptr(ReferenceRotation);
%include <OpenSim/Simulation/MarkersReference.h>
// Since we have both MarkersReference and shared_ptr<MarkersReference>
// Across the interface, DO NOT use %shared_ptr macro here as it treats
// all pointers and references as std::shared_ptr
//
%template(SharedMarkersReference) std::shared_ptr<OpenSim::MarkersReference>;
%template(SetMarkerWeights) OpenSim::Set<MarkerWeight, OpenSim::Object>;
%include <OpenSim/Simulation/CoordinateReference.h>
%include <OpenSim/Simulation/OrientationsReference.h>
// Since we have both OrientationsReference and shared_ptr<OrientationsReference>
// Across the interface, DO NOT use %shared_ptr macro here as it treats
// all pointers and references as std::shared_ptr
//
%template (SetOientationWeights) OpenSim::Set<OrientationWeight, OpenSim::Object>;
%template(SharedOrientationsReference) std::shared_ptr<OpenSim::OrientationsReference>;
%include <OpenSim/Simulation/BufferedOrientationsReference.h>
%shared_ptr(OpenSim::BufferedOrientationsReference);

%include <OpenSim/Simulation/AssemblySolver.h>
%include <OpenSim/Simulation/InverseKinematicsSolver.h>
%include <OpenSim/Simulation/OpenSense/IMUPlacer.h>
%include <OpenSim/Simulation/OpenSense/IMU.h>

%include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>

%template(StdVectorIMUs) std::vector< OpenSim::IMU* >;

%include <OpenSim/Simulation/StatesDocument.h>
%include <OpenSim/Simulation/StatesTrajectory.h>
// This enables iterating using the getBetween() method.
%template(IteratorRangeStatesTrajectoryIterator)
    SimTK::IteratorRange<OpenSim::StatesTrajectory::const_iterator>;
%include <OpenSim/Simulation/StatesTrajectoryReporter.h>
%include <OpenSim/Simulation/PositionMotion.h>
%include <OpenSim/Simulation/SimulationUtilities.h>
%template(analyze) OpenSim::analyze<double>;
%template(analyzeVec3) OpenSim::analyze<SimTK::Vec3>;
%template(analyzeSpatialVec) OpenSim::analyze<SimTK::SpatialVec>;
%template(analyzeRotation) OpenSim::analyze<SimTK::Rotation_<double>>;

%include <OpenSim/Simulation/VisualizerUtilities.h>

%include <OpenSim/Simulation/TableProcessor.h>

// Iterators.
%template(FrameList) OpenSim::ComponentList<const OpenSim::Frame>;
%template(FrameIterator) OpenSim::ComponentListIterator<const OpenSim::Frame>;

%template(BodyList) OpenSim::ComponentList<const OpenSim::Body>;
%template(BodyIterator) OpenSim::ComponentListIterator<const OpenSim::Body>;

%template(MuscleList) OpenSim::ComponentList<const OpenSim::Muscle>;
%template(MuscleIterator) OpenSim::ComponentListIterator<const OpenSim::Muscle>;

%template(ModelComponentList) OpenSim::ComponentList<const OpenSim::ModelComponent>;
%template(ModelComponentIterator) OpenSim::ComponentListIterator<const OpenSim::ModelComponent>;

%template(JointList) OpenSim::ComponentList<const OpenSim::Joint>;
%template(JointIterator) OpenSim::ComponentListIterator<const OpenSim::Joint>;

%template(ActuatorList) OpenSim::ComponentList<const OpenSim::Actuator>;
%template(ActuatorIterator) OpenSim::ComponentListIterator<const OpenSim::Actuator>;

%template(Thelen2003MuscleList)
    OpenSim::ComponentList<const OpenSim::Thelen2003Muscle>;
%template(Thelen2003MuscleIterator)
    OpenSim::ComponentListIterator<const OpenSim::Thelen2003Muscle>;

%template(Millard2012EquilibriumMuscleList)
    OpenSim::ComponentList<const OpenSim::Millard2012EquilibriumMuscle>;
%template(Millard2012EquilibriumMuscleIterator)
    OpenSim::ComponentListIterator<const OpenSim::Millard2012EquilibriumMuscle>;

%extend OpenSim::Model {
    OpenSim::ComponentList<const OpenSim::Frame> getFrameList(){
        return $self->getComponentList<OpenSim::Frame>();
    }
    OpenSim::ComponentList<const OpenSim::Body> getBodyList(){
        return $self->getComponentList<OpenSim::Body>();
    }
    OpenSim::ComponentList<const OpenSim::Muscle> getMuscleList(){
        return $self->getComponentList<OpenSim::Muscle>();
    }
    OpenSim::ComponentList<const OpenSim::Joint> getJointList(){
        return $self->getComponentList<OpenSim::Joint>();
    }
    OpenSim::ComponentList<const OpenSim::Actuator> getActuatorList(){
        return $self->getComponentList<OpenSim::Actuator>();
    }
    OpenSim::ComponentList<const OpenSim::ModelComponent> getModelComponentList(){
        return $self->getComponentList<OpenSim::ModelComponent>();
    }
    OpenSim::ComponentList<const OpenSim::Thelen2003Muscle> getThelen2003MuscleList(){
        return $self->getComponentList<OpenSim::Thelen2003Muscle>();
    }
    OpenSim::ComponentList<const OpenSim::Millard2012EquilibriumMuscle> getMillard2012EquilibriumMuscleList(){
        return $self->getComponentList<OpenSim::Millard2012EquilibriumMuscle>();
    }
}

%include <OpenSim/Actuators/osimActuatorsDLL.h>
%include <OpenSim/Actuators/ActiveForceLengthCurve.h>
%include <OpenSim/Actuators/FiberCompressiveForceCosPennationCurve.h>
%include <OpenSim/Actuators/FiberCompressiveForceLengthCurve.h>
%include <OpenSim/Actuators/FiberForceLengthCurve.h>
%include <OpenSim/Actuators/ForceVelocityCurve.h>
%include <OpenSim/Actuators/ForceVelocityInverseCurve.h>
%include <OpenSim/Actuators/TendonForceLengthCurve.h>
%include <OpenSim/Actuators/MuscleFirstOrderActivationDynamicModel.h>
%include <OpenSim/Actuators/MuscleFixedWidthPennationModel.h>
%include <OpenSim/Actuators/Thelen2003Muscle.h>
%include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>


// Compensate for insufficient C++11 support in SWIG
// =================================================
/*
Extend concrete Joints to use the inherited base constructors.
This is only necessary because SWIG does not generate these inherited
constructors provided by C++11's 'using' (e.g. using Joint::Joint) declaration.
Note that CustomJoint and EllipsoidJoint do implement their own
constructors because they have additional arguments.
*/
%define EXPOSE_JOINT_CONSTRUCTORS_HELPER(NAME)
%extend OpenSim::NAME {
    NAME() {
        return new NAME();
    }
    NAME(const std::string& name,
         const PhysicalFrame& parent,
         const PhysicalFrame& child) {
        return new NAME(name, parent, child);
    }

    NAME(const std::string& name,
         const PhysicalFrame& parent,
         const SimTK::Vec3& locationInParent,
         const SimTK::Vec3& orientationInParent,
         const PhysicalFrame& child,
         const SimTK::Vec3& locationInChild,
         const SimTK::Vec3& orientationInChild) {
        return new NAME(name, parent, locationInParent, orientationInParent,
                    child, locationInChild, orientationInChild);
    }
};
%enddef

EXPOSE_JOINT_CONSTRUCTORS_HELPER(FreeJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(BallJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(PinJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(SliderJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(WeldJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(GimbalJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(UniversalJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(PlanarJoint);

%extend OpenSim::PhysicalOffsetFrame {
    PhysicalOffsetFrame() {
        return new PhysicalOffsetFrame();
    }
    PhysicalOffsetFrame(const PhysicalFrame& parent,
                        const SimTK::Transform& offset) {
        return new PhysicalOffsetFrame(parent, offset);
    }
    PhysicalOffsetFrame(const std::string& name,
                const PhysicalFrame& parent,
                const SimTK::Transform& offset) {
        return new PhysicalOffsetFrame(name, parent, offset);
    }

    PhysicalOffsetFrame(const std::string& name,
                const std::string& parentName,
                const SimTK::Transform& offset) {
        return new PhysicalOffsetFrame(name, parentName, offset);
    }
};

EXPOSE_SET_CONSTRUCTORS_HELPER(ModelComponentSet);
EXPOSE_SET_CONSTRUCTORS_HELPER(BodySet);
EXPOSE_SET_CONSTRUCTORS_HELPER(JointSet);
EXPOSE_SET_CONSTRUCTORS_HELPER(ConstraintSet);
EXPOSE_SET_CONSTRUCTORS_HELPER(ForceSet);
EXPOSE_SET_CONSTRUCTORS_HELPER(ControllerSet);
EXPOSE_SET_CONSTRUCTORS_HELPER(ContactGeometrySet);
EXPOSE_SET_CONSTRUCTORS_HELPER(PathPointSet);
EXPOSE_SET_CONSTRUCTORS_HELPER(ProbeSet);
EXPOSE_SET_CONSTRUCTORS_HELPER(MarkerSet);
EXPOSE_SET_CONSTRUCTORS_HELPER(WrapObjectSet);
EXPOSE_SET_CONSTRUCTORS_HELPER(CoordinateSet);

