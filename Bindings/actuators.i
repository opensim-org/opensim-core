%include <OpenSim/Simulation/Model/Actuator.h>
%template(SetActuators) OpenSim::Set<OpenSim::Actuator>;
%template(SetAnalysis) OpenSim::Set<OpenSim::Analysis>;
%include <OpenSim/Simulation/Model/AnalysisSet.h>

//osimActuators
%include <OpenSim/Actuators/osimActuatorsDLL.h>
%include <OpenSim/Actuators/CoordinateActuator.h>
%include <OpenSim/Actuators/PointActuator.h>
%include <OpenSim/Actuators/TorqueActuator.h>
%include <OpenSim/Actuators/BodyActuator.h>
%include <OpenSim/Actuators/PointToPointActuator.h>
%include <OpenSim/Actuators/ClutchedPathSpring.h>
%include <OpenSim/Actuators/SpringGeneralizedForce.h>
%include <OpenSim/Actuators/RigidTendonMuscle.h>
%include <OpenSim/Actuators/ActiveForceLengthCurve.h>
%include <OpenSim/Actuators/FiberCompressiveForceCosPennationCurve.h>
%include <OpenSim/Actuators/FiberCompressiveForceLengthCurve.h>
%include <OpenSim/Actuators/FiberForceLengthCurve.h>
%include <OpenSim/Actuators/ForceVelocityCurve.h>
%include <OpenSim/Actuators/ForceVelocityInverseCurve.h>
%include <OpenSim/Actuators/TendonForceLengthCurve.h>
%include <OpenSim/Actuators/ClutchedPathSpring.h>
%include <OpenSim/Actuators/MuscleFirstOrderActivationDynamicModel.h>
%include <OpenSim/Actuators/MuscleFixedWidthPennationModel.h>

%include <OpenSim/Actuators/Thelen2003Muscle.h>
%include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
%include <OpenSim/Actuators/Millard2012AccelerationMuscle.h>
