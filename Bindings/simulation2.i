%template(SetModelComponents) OpenSim::Set<OpenSim::ModelComponent>;

%template(ModelComponentSetModelComponent) 
    OpenSim::ModelComponentSet<OpenSim::ModelComponent>;

%template(SetMuscles) OpenSim::Set<OpenSim::Muscle>;

// Following three lines hacked in out of order to work around WrapObjects 
// use in PhysicalFrame.
%include <OpenSim/Simulation/Wrap/WrapObject.h>
%template(SetWrapObject) OpenSim::Set<OpenSim::WrapObject>;
%include <OpenSim/Simulation/Wrap/WrapObjectSet.h>

%include <OpenSim/Simulation/Model/OffsetFrame.h>
%template(PhysicalFrameWithOffset) 
    OpenSim::OffsetFrame<OpenSim::PhysicalFrame>; 
%include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
%template(SetFrames) OpenSim::Set<OpenSim::Frame>;
%template(ModelComponentSetFrames) OpenSim::ModelComponentSet<OpenSim::Frame>;
%include <OpenSim/Simulation/Model/FrameSet.h>

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
%template(ModelComponentSetCoordinates) 
    OpenSim::ModelComponentSet<OpenSim::Coordinate>;
%include <OpenSim/Simulation/Model/CoordinateSet.h>

%template(SetJoints) OpenSim::Set<OpenSim::Joint>;
%template(ModelComponentSetJoints) OpenSim::ModelComponentSet<OpenSim::Joint>;
%include <OpenSim/Simulation/Model/JointSet.h>

%template(SetConstraints) OpenSim::Set<OpenSim::Constraint>;
%template(ModelComponentSetConstraints) 
    OpenSim::ModelComponentSet<OpenSim::Constraint>;
%include <OpenSim/Simulation/Model/ConstraintSet.h>

%template(SetForces) OpenSim::Set<OpenSim::Force>;
%template(ModelComponentSetForces) OpenSim::ModelComponentSet<OpenSim::Force>;
%include <OpenSim/Simulation/Model/ForceSet.h>
%include <OpenSim/Simulation/Model/ExternalForce.h>
%template(SetExternalForces) OpenSim::Set<OpenSim::ExternalForce>;

%include <OpenSim/Simulation/Model/TwoFrameLinker.h>
%template(TwoFrameLinkerForce) 
    OpenSim::TwoFrameLinker<OpenSim::Force, OpenSim::PhysicalFrame>;
%template(TwoFrameLinkerConstraint) 
    OpenSim::TwoFrameLinker<OpenSim::Constraint, OpenSim::PhysicalFrame>;

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

%template(SetControllers) OpenSim::Set<OpenSim::Controller>;
%template(ModelComponentSetControllers) 
    OpenSim::ModelComponentSet<OpenSim::Controller>;
%include <OpenSim/Simulation/Model/ControllerSet.h>
%template(ModelComponentSetExternalForces) 
    OpenSim::ModelComponentSet<OpenSim::ExternalForce>;
%include <OpenSim/Simulation/Model/ExternalLoads.h>
%include <OpenSim/Simulation/Model/PrescribedForce.h>
%include <OpenSim/Simulation/Model/CoordinateLimitForce.h>

%template(SetContactGeometry) OpenSim::Set<OpenSim::ContactGeometry>;
%template(ModelComponentSetContactGeometry) 
    OpenSim::ModelComponentSet<OpenSim::ContactGeometry>;
%include <OpenSim/Simulation/Model/ContactGeometrySet.h>
%include <OpenSim/Simulation/Model/ContactHalfSpace.h>
%include <OpenSim/Simulation/Model/ContactMesh.h>
%include <OpenSim/Simulation/Model/ContactSphere.h>
%include <OpenSim/Simulation/Model/ElasticFoundationForce.h>
%include <OpenSim/Simulation/Model/HuntCrossleyForce.h>

