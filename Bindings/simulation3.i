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

//%include <OpenSim/Simulation/Wrap/WrapObject.h>
%include <OpenSim/Simulation/Wrap/WrapSphere.h>
%include <OpenSim/Simulation/Wrap/WrapCylinder.h>
%include <OpenSim/Simulation/Wrap/WrapTorus.h>
%include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
//%template(SetWrapObject) OpenSim::Set<OpenSim::WrapObject>;
//%include <OpenSim/Simulation/Wrap/WrapObjectSet.h>
%include <OpenSim/Simulation/Wrap/PathWrap.h>
%template(SetPathWrap) OpenSim::Set<OpenSim::PathWrap>;
%include <OpenSim/Simulation/Wrap/PathWrapSet.h>
%include <OpenSim/Simulation/Wrap/WrapCylinderObst.h>
%include <OpenSim/Simulation/Wrap/WrapSphereObst.h>
%include <OpenSim/Simulation/Wrap/WrapDoubleCylinderObst.h>

%include <OpenSim/Simulation/Model/PathPoint.h>
%include <OpenSim/Simulation/Wrap/PathWrapPoint.h>
%include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
%include <OpenSim/Simulation/Model/MovingPathPoint.h>
%template(SetPathPoint) OpenSim::Set<OpenSim::PathPoint>;
%template(ArrayPathPoint) OpenSim::Array<OpenSim::PathPoint*>;
%include <OpenSim/Simulation/Model/PathPointSet.h>

%include <OpenSim/Simulation/Model/PointForceDirection.h>
%template(ArrayPointForceDirection) 
    OpenSim::Array<OpenSim::PointForceDirection*>;

%include <OpenSim/Simulation/Model/GeometryPath.h>
%include <OpenSim/Simulation/Model/Ligament.h>
%include <OpenSim/Simulation/Model/PathActuator.h>
%include <OpenSim/Simulation/Model/Muscle.h>
%include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>
%include <OpenSim/Simulation/Model/PointToPointSpring.h>
%include <OpenSim/Simulation/Model/ExpressionBasedPointToPointForce.h>
%include <OpenSim/Simulation/Model/ExpressionBasedCoordinateForce.h>

%include <OpenSim/Simulation/Model/PathSpring.h>
%include <OpenSim/Simulation/Model/BushingForce.h>
%include <OpenSim/Simulation/Model/FunctionBasedBushingForce.h>
%include <OpenSim/Simulation/Model/ExpressionBasedBushingForce.h>

%include <OpenSim/Simulation/Solver.h>
%include <OpenSim/Simulation/Reference.h>

%template(ReferenceVec3) OpenSim::Reference_<SimTK::Vec3>;
%template(ReferenceDouble) OpenSim::Reference_<double>;
%template(ArrayCoordinateReference) SimTK::Array_<OpenSim::CoordinateReference>;

%include <OpenSim/Simulation/MarkersReference.h>
%include <OpenSim/Simulation/CoordinateReference.h>
%include <OpenSim/Simulation/AssemblySolver.h>
%include <OpenSim/Simulation/InverseKinematicsSolver.h>

%include <OpenSim/Simulation/StatesTrajectory.h>
// This enables iterating using the getBetween() method.
%template(IteratorRangeStatesTrajectoryIterator) 
    SimTK::IteratorRange<OpenSim::StatesTrajectory::const_iterator>;

