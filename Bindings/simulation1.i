// osimSimulation
%include <OpenSim/Simulation/osimSimulationDLL.h>

%typedef SimTK::DecorativeGeometry::Representation VisualRepresentation;

%include <OpenSim/Common/Component.h>
%include <OpenSim/Simulation/Model/Appearance.h>
%include <OpenSim/Simulation/Model/Geometry.h>
%include <OpenSim/Simulation/Model/ModelComponent.h>
%template(SetModelComponents) OpenSim::Set<OpenSim::ModelComponent>;
%include <OpenSim/Simulation/Model/ModelComponentSet.h>
%template(ModelComponentSetModelComponent) OpenSim::ModelComponentSet<OpenSim::ModelComponent>;
%include <OpenSim/Simulation/Model/ComponentSet.h>

%template(SetMuscles) OpenSim::Set<OpenSim::Muscle>;

%include <OpenSim/Simulation/Solver.h>
%include <OpenSim/Simulation/InverseDynamicsSolver.h>
%include <OpenSim/Simulation/MomentArmSolver.h>

%include <OpenSim/Simulation/Model/Frame.h>
// Following three lines hacked in out of order to work around WrapObjects use in PhysicalFrame
%include <OpenSim/Simulation/Wrap/WrapObject.h>
%template(SetWrapObject) OpenSim::Set<OpenSim::WrapObject>;
%include <OpenSim/Simulation/Wrap/WrapObjectSet.h>

%include <OpenSim/Simulation/Model/PhysicalFrame.h>
%include <OpenSim/Simulation/Model/Ground.h>
%include <OpenSim/Simulation/Model/OffsetFrame.h>
%template(PhysicalFrameWithOffset)   OpenSim::OffsetFrame<OpenSim::PhysicalFrame>; 
%include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
%template(SetFrames) OpenSim::Set<OpenSim::Frame>;
%template(ModelComponentSetFrames) OpenSim::ModelComponentSet<OpenSim::Frame>;
%include <OpenSim/Simulation/Model/FrameSet.h>

%include <OpenSim/Simulation/SimbodyEngine/Body.h>
%template(SetBodies) OpenSim::Set<OpenSim::Body>;
%template(ModelComponentSetBodies) OpenSim::ModelComponentSet<OpenSim::Body>;
%include <OpenSim/Simulation/Model/BodySet.h>

%include <OpenSim/Simulation/Model/BodyScale.h>
%template(SetBodyScales) OpenSim::Set<OpenSim::BodyScale>;
%include <OpenSim/Simulation/Model/BodyScaleSet.h>

%include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
%include <OpenSim/Simulation/SimbodyEngine/TransformAxis.h>
%include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
%include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
%template(SetCoordinates) OpenSim::Set<OpenSim::Coordinate>;
%template(ModelComponentSetCoordinates) OpenSim::ModelComponentSet<OpenSim::Coordinate>;
%include <OpenSim/Simulation/Model/CoordinateSet.h>

%include <OpenSim/Simulation/SimbodyEngine/Joint.h>
%template(SetJoints) OpenSim::Set<OpenSim::Joint>;
%template(ModelComponentSetJoints) OpenSim::ModelComponentSet<OpenSim::Joint>;
%include <OpenSim/Simulation/Model/JointSet.h>

%include <OpenSim/Simulation/SimbodyEngine/Constraint.h>
%template(SetConstraints) OpenSim::Set<OpenSim::Constraint>;
%template(ModelComponentSetConstraints) OpenSim::ModelComponentSet<OpenSim::Constraint>;
%include <OpenSim/Simulation/Model/ConstraintSet.h>

%include <OpenSim/Simulation/Model/Force.h>
%template(SetForces) OpenSim::Set<OpenSim::Force>;
%template(ModelComponentSetForces) OpenSim::ModelComponentSet<OpenSim::Force>;
%include <OpenSim/Simulation/Model/ForceSet.h>
%include <OpenSim/Simulation/Model/ExternalForce.h>
%template(SetExternalForces) OpenSim::Set<OpenSim::ExternalForce>;

%include <OpenSim/Simulation/Model/TwoFrameLinker.h>
%template(TwoFrameLinkerForce) OpenSim::TwoFrameLinker<OpenSim::Force, OpenSim::PhysicalFrame>;
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

%include <OpenSim/Simulation/SimbodyEngine/WeldConstraint.h>
%include <OpenSim/Simulation/SimbodyEngine/PointConstraint.h>
%include <OpenSim/Simulation/SimbodyEngine/ConstantDistanceConstraint.h>
%include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>
%include <OpenSim/Simulation/SimbodyEngine/PointOnLineConstraint.h>

%include <OpenSim/Simulation/Control/Controller.h>
%template(SetControllers) OpenSim::Set<OpenSim::Controller>;
%template(ModelComponentSetControllers) OpenSim::ModelComponentSet<OpenSim::Controller>;
%include <OpenSim/Simulation/Model/ControllerSet.h>

%template(ModelComponentSetExternalForces) OpenSim::ModelComponentSet<OpenSim::ExternalForce>;
%include <OpenSim/Simulation/Model/ExternalLoads.h>
%include <OpenSim/Simulation/Model/PrescribedForce.h>
%include <OpenSim/Simulation/Model/CoordinateLimitForce.h>

%include <OpenSim/Simulation/Model/ContactGeometry.h>
%template(SetContactGeometry) OpenSim::Set<OpenSim::ContactGeometry>;
%template(ModelComponentSetContactGeometry) OpenSim::ModelComponentSet<OpenSim::ContactGeometry>;
%include <OpenSim/Simulation/Model/ContactGeometrySet.h>
%include <OpenSim/Simulation/Model/ContactHalfSpace.h>
%include <OpenSim/Simulation/Model/ContactMesh.h>
%include <OpenSim/Simulation/Model/ContactSphere.h>
%include <OpenSim/Simulation/Model/ElasticFoundationForce.h>
%include <OpenSim/Simulation/Model/HuntCrossleyForce.h>

%include <OpenSim/Simulation/Model/Actuator.h>
%template(SetActuators) OpenSim::Set<OpenSim::Actuator>;
%include <OpenSim/Simulation/Model/Analysis.h>
%template(SetAnalysis) OpenSim::Set<OpenSim::Analysis>;
%include <OpenSim/Simulation/Model/AnalysisSet.h>

%include <OpenSim/Simulation/Control/Control.h>
%template(SetControls) OpenSim::Set<OpenSim::Control>;
%include <OpenSim/Simulation/Control/ControlSet.h>
%include <OpenSim/Simulation/Control/ControlConstant.h>
%include <OpenSim/Simulation/Control/ControlLinearNode.h>
%template(SetControlNodes) OpenSim::ArrayPtrs<OpenSim::ControlLinearNode>;
%include <OpenSim/Simulation/Control/ControlLinear.h>
%include <OpenSim/Simulation/Control/Controller.h>
%include <OpenSim/Simulation/Control/PrescribedController.h>

%include <OpenSim/Simulation/Manager/Manager.h>
%include <OpenSim/Simulation/Model/AbstractTool.h>
%include <OpenSim/Simulation/Model/Station.h>
%include <OpenSim/Simulation/Model/Marker.h>
%template(SetMarkers) OpenSim::Set<OpenSim::Marker>;
%template(ModelComponentSetMarkers) OpenSim::ModelComponentSet<OpenSim::Marker>;
%include <OpenSim/Simulation/Model/MarkerSet.h>

%include <OpenSim/Simulation/Model/Probe.h>
%template(SetProbes) OpenSim::Set<OpenSim::Probe>;
%template(ModelComponentSetProbes) OpenSim::ModelComponentSet<OpenSim::Probe>;
%include <OpenSim/Simulation/Model/ProbeSet.h>
%include <OpenSim/Simulation/Model/SystemEnergyProbe.h>
%include <OpenSim/Simulation/Model/JointInternalPowerProbe.h>
%include <OpenSim/Simulation/Model/ActuatorPowerProbe.h>
%include <OpenSim/Simulation/Model/ActuatorForceProbe.h>
%include <OpenSim/Simulation/Model/MuscleActiveFiberPowerProbe.h>
%include <OpenSim/Simulation/Model/Bhargava2004MuscleMetabolicsProbe.h>
%include <OpenSim/Simulation/Model/Umberger2010MuscleMetabolicsProbe.h>
%include <OpenSim/Simulation/Model/ModelDisplayHints.h>
%include <OpenSim/Simulation/Model/ModelVisualPreferences.h>
%include <OpenSim/Simulation/Model/ModelVisualizer.h>
%include <OpenSim/Simulation/Model/Model.h>

// Iterators.
// TODO rename to singular form.
%template(FrameList)          OpenSim::ComponentList<OpenSim::Frame>;
%template(BodyList)           OpenSim::ComponentList<OpenSim::Body>;
%template(MuscleList)         OpenSim::ComponentList<OpenSim::Muscle>;
%template(ModelComponentList) OpenSim::ComponentList<OpenSim::ModelComponent>;
%template(JointList)          OpenSim::ComponentList<OpenSim::Joint>;

%template(getComponentsList) 
    OpenSim::Component::getComponentList<OpenSim::Component>;
%template(getFrameList) 
    OpenSim::Component::getComponentList<OpenSim::Frame>;
%template(getBodyList) 
    OpenSim::Component::getComponentList<OpenSim::Body>;
%template(getMuscleList) 
    OpenSim::Component::getComponentList<OpenSim::Muscle>;
%template(getModelComponentList) 
    OpenSim::Component::getComponentList<OpenSim::ModelComponent>;
%template(getJointList) 
    OpenSim::Component::getComponentList<OpenSim::Joint>;

%template(FrameIterator) 
    OpenSim::ComponentListIterator<OpenSim::Frame>;
%template(BodyIterator) 
    OpenSim::ComponentListIterator<OpenSim::Body>;
%template(MuscleIterator) 
    OpenSim::ComponentListIterator<OpenSim::Muscle>;
%template(ModelComponentIterator) 
    OpenSim::ComponentListIterator<OpenSim::ModelComponent>;
%template(JointIterator) 
    OpenSim::ComponentListIterator<OpenSim::Joint>;

