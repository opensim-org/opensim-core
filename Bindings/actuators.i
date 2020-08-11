//osimActuators
%include <OpenSim/Actuators/osimActuatorsDLL.h>
%include <OpenSim/Actuators/CoordinateActuator.h>
%include <OpenSim/Actuators/ActivationCoordinateActuator.h>
%include <OpenSim/Actuators/PointActuator.h>
%include <OpenSim/Actuators/TorqueActuator.h>
%include <OpenSim/Actuators/BodyActuator.h>
%include <OpenSim/Actuators/PointToPointActuator.h>
%include <OpenSim/Actuators/ClutchedPathSpring.h>
%include <OpenSim/Actuators/SpringGeneralizedForce.h>
%include <OpenSim/Actuators/RigidTendonMuscle.h>
%include <OpenSim/Actuators/Millard2012AccelerationMuscle.h>
%include <OpenSim/Actuators/McKibbenActuator.h>
%include <OpenSim/Actuators/DeGrooteFregly2016Muscle.h>

%include <OpenSim/Actuators/ModelFactory.h>

namespace OpenSim {
    %ignore ModelProcessor::setModel(std::unique_ptr<Model>);
}

%extend OpenSim::ModelProcessor {
    void setModel(Model* model) {
        $self->setModel(std::unique_ptr<Model>(model));
    }
};

%include <OpenSim/Actuators/ModelProcessor.h>
%include <OpenSim/Actuators/ModelOperators.h>
