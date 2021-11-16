#ifndef OPENSIM_COMAK_TARGET_H_
#define OPENSIM_COMAK_TARGET_H_
/* -------------------------------------------------------------------------- *
 *                              COMAKTarget.h                                 *
 * -------------------------------------------------------------------------- *
 * Author(s): Colin Smith                                                     *
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

//=============================================================================
// INCLUDES
//=============================================================================
//#include "osimAnalysesDLL.h"
#include "OpenSim/Common/Array.h"
#include "COMAKTool.h"
#include <simmath/Optimizer.h>

//=============================================================================
//=============================================================================
namespace OpenSim { 

/**

 */
class ComakTarget : public SimTK::OptimizerSystem
{


//=============================================================================
// METHODS
//=============================================================================
public:
    ComakTarget(SimTK::State s, Model* aModel,
        const SimTK::Vector& observed_udot,
        const SimTK::Vector& init_parameters,
        double dt, double unit_udot_epsilon, 
        Array<std::string> primary_coords, Array<std::string> secondary_coords,
        Array<std::string> muscle_path,
        Array<std::string> non_muscle_actuator_path,
        const SimTK::Vector& max_change);

    //-------------------------------------------------------------------------
    // REQUIRED OPTIMIZATION TARGET METHODS
    //-------------------------------------------------------------------------
    
    /**
     * Compute objective function value (peformance) from the optimization 
     * parameters
     *
     * @param parameters Vector of optimization parameters.
     * @param performance Value of the performance criterion.
     * @return Status (normal termination = 0, error < 0).
     */
    int objectiveFunc(const SimTK::Vector &parameters, bool new_parameters, 
        SimTK::Real& performance) const override;

    
    /**
     * Compute the gradient of performance given parameters.
     *
     * @param parameters Vector of optimization parameters.
     * @param gradient Derivatives of performance with respect to the parameters.
     * @return Status (normal termination = 0, error < 0).
     */
    int gradientFunc(const SimTK::Vector &parameters, bool new_parameters,
        SimTK::Vector &gradient) const override;

    /**
     * Compute the optimization constraints given parameters.
     *
     * @param parameters Vector of optimization parameters.
     * @param constraints Vector of optimization constraints.
     * @return Status (normal termination = 0, error < 0).
     */
    int constraintFunc(const SimTK::Vector &parameters, bool new_parameters, 
        SimTK::Vector &constraints) const override;

    /**
     * Compute the gradient of constraints given parameters.
     *
     * @param parameters Vector of parameters.
     * @param jac Derivative of constraint with respect to the parameters.
     * @return Status (normal termination = 0, error < 0).
     */
    int constraintJacobian(const SimTK::Vector &parameters, 
        bool new_parameters, SimTK::Matrix &jac) const override;

    //Helper
    void initialize();
    void realizeAccelerationFromParameters(
        SimTK::State& s, const SimTK::Vector &parameters);
    void precomputeConstraintMatrix();
    void setParameterBounds(double scale);
    void printPerformance(SimTK::Vector parameters);

    //Set
   void setModel(Model& aModel){
        _model = &aModel;
    }

    void setCostFunctionWeight(SimTK::Vector weight) {
        _muscle_weight = weight;
    }

    void setDesiredActivation(SimTK::Vector desired_act) {
        _desired_act = desired_act;
    }

    void setMaxChange(SimTK::Vector max_change) {
        _max_change = max_change;
    }

    void setUnitUdotEpsilon(double unit_udot_epsilon) {
        _unit_udot_epsilon = unit_udot_epsilon;
    };

    void setDT(double dt) {
        _dt = dt;
    };

    void setOptimalForces(SimTK::Vector optimal_force) {
         _optimal_force = optimal_force;
    }

    void setMuscleVolumes(SimTK::Vector msl_volumes) {
        _muscle_volumes = msl_volumes;
    }

    void setContactEnergyWeight(double contact_energy_weight) {
        _contact_energy_weight = contact_energy_weight;
    }

    void setNonMuscleActuatorWeight(double non_muscle_actuator_weight) {
        _non_muscle_actuator_weight = non_muscle_actuator_weight;
    }

    void setActivationExponent(double activation_exponent) {
        _activation_exponent = activation_exponent;
    }

    void setScaleDeltaCoordinates(double scale) {
        _scale_delta_coord = scale;
    }

    void setParameterNames(const Array<std::string>& parameter_names) {
        _parameter_names = parameter_names;
    }

    //=========================================================================
    // DATA
    //=========================================================================
private:
    Model *_model;
    SimTK::State _state;
    SimTK::Vector _optimal_force;
    const SimTK::Vector _init_parameters;
    double _activation_exponent;
    SimTK::Vector _observed_udot;

    int _nSecondaryCoord;
    int _nPrimaryCoord;
    int _nCoordinates;
    int _nActuators;

    int _nConstraints;
    Array<std::string> _constraint_names;

    int _nParameters;
    Array<std::string> _parameter_names;

    int _nMuscles;
    Array<std::string> _muscle_path;
    SimTK::Vector _muscle_volumes;
    SimTK::Vector _muscle_weight;
    SimTK::Vector _desired_act;

    int _nNonMuscleActuators;
    Array<std::string> _non_muscle_actuator_path;

    SimTK::Matrix _msl_unit_udot;
    SimTK::Matrix _non_muscle_actuator_unit_udot;
    SimTK::Matrix _secondary_coord_unit_udot;
    SimTK::Matrix _secondary_speed_unit_udot;
    SimTK::Vector _secondary_coord_unit_energy;

    Array<std::string> _primary_coords;
    Array<std::string> _secondary_coords;

    SimTK::Vector _max_change;
    double _unit_udot_epsilon;
    double _dt;
    double _initial_contact_energy;
    double _contact_energy_weight;
    double _non_muscle_actuator_weight;
    
    SimTK::Vector _initial_udot;
    SimTK::Vector _constraint_initial_udot;
    SimTK::Vector _constraint_desired_udot;

    double _scale_delta_coord;

    SimTK::Matrix _constraint_matrix;
protected:
};

}; //namespace

#endif // OPENSIM_COMAK_TARGET_H_
