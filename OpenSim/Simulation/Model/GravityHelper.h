#ifndef OPENSIM_GRAVITY_HELPER_H_
#define OPENSIM_GRAVITY_HELPER_H_
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "Simbody.h"
namespace OpenSim {

class  GravityHelper {
    SimTK::ReferencePtr<Model> _model;
public:
    GravityHelper(OpenSim::Model& aModel){
        _model = aModel;
    }
    const SimTK::Vector_<SimTK::SpatialVec>& getGravityForces(
            const SimTK::State& state)
    {
        return _model->getGravityForce().getBodyForces(state);
    }

};
}

#endif // OPENSIM_GRAVITY_HELPER_H_