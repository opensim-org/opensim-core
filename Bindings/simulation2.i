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

