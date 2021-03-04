/* -------------------------------------------------------------------------- *
 *                               COMAKTarget.cpp                              *
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
#include <OpenSim/Simulation/Model/Model.h>
#include "COMAKTarget.h"
#include "OpenSim/Simulation/Model/Smith2018ArticularContactForce.h"
//#include <OpenSim.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR
//=============================================================================

ComakTarget::
ComakTarget(SimTK::State s, Model* aModel,
    const SimTK::Vector& observed_udot,
    const SimTK::Vector& init_parameters, 
    double dt, double unit_udot_epsilon,
    Array<std::string> primary_coords, Array<std::string> secondary_coords,
    Array<std::string> muscle_path,
    Array<std::string> non_muscle_actuator_path,
    const SimTK::Vector& max_change) : _init_parameters(init_parameters)
{
    // Set Parameters
    _model = aModel;
    _state = s;

    _primary_coords = primary_coords;
    _nPrimaryCoord = primary_coords.size();

    _secondary_coords = secondary_coords;
    _nSecondaryCoord = secondary_coords.size();

    _muscle_path = muscle_path;
    _non_muscle_actuator_path = non_muscle_actuator_path;

    _observed_udot = observed_udot;

    _max_change = max_change;
    _unit_udot_epsilon = unit_udot_epsilon;
    _dt = dt;

    // Set Defaults
    _activation_exponent = 2.0;
    _scale_delta_coord = 1.0;
    _contact_energy_weight = 0.0;
    _non_muscle_actuator_weight = 1000;

}

void ComakTarget::initialize(){
    //Number of Parameters
    _nMuscles = _muscle_path.size();
    _nNonMuscleActuators = _non_muscle_actuator_path.size();
    _nActuators = _nMuscles + _nNonMuscleActuators;

    _nParameters = _nActuators + _nSecondaryCoord;
    setNumParameters(_nParameters);
    setParameterBounds(1);
    
    //Number of Constraints
    int nC = 0;
    int nCoord = 0;
    for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
        if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1) {

            _constraint_names.append(coord.getName());
            nC++;
        }
        else if(
            _secondary_coords.findIndex(coord.getAbsolutePathString())> -1) {

            _constraint_names.append(coord.getName());
            nC++;
        }
        nCoord++;
    }
    _nConstraints = nC;
    _nCoordinates = nCoord;

    setNumEqualityConstraints(nC);
    setNumLinearEqualityConstraints(nC);
    setNumInequalityConstraints(0);

    if(_muscle_weight.size() == 0){
        _muscle_weight.resize(_nMuscles);
        _muscle_weight = 1.0;
    }
    if(_desired_act.size() == 0){
        _desired_act.resize(_nMuscles);
        _desired_act = 0.0;
    }
    if(_muscle_volumes.size() == 0){
        _muscle_volumes.resize(_nMuscles);
        _muscle_volumes = 1.0;
    }
    if (_optimal_force.size() == 0) {
        _optimal_force.resize(_nActuators);
        _optimal_force = 1.0;
    }
    if (_parameter_names.size() == 0) {
        _parameter_names.setSize(_nParameters);       
    }

    //Precompute Constraint Matrix
    precomputeConstraintMatrix();

    if (_verbose > 4) {
        std::cout << std::endl;
        std::cout << "Num Parameters: " 
            << _nParameters << std::endl;
        std::cout << "Num Constraints: " 
            << _nConstraints << std::endl;
        std::cout << "Num Actuators: " 
            << _nActuators << std::endl;
        std::cout << "Num Coordinates: " 
            << _nCoordinates << std::endl;
        std::cout << "Num Secondary Coordinates: " 
            << _secondary_coords.size() << std::endl;
        std::cout << "Num Primary Coordinates: " 
            << _primary_coords.size() << std::endl;
        std::cout << std::endl;
    }
}

void ComakTarget::precomputeConstraintMatrix() {
    _constraint_initial_udot.resize(_nConstraints);
    _constraint_desired_udot.resize(_nConstraints);

    //Calculate the initial udots generated by the initial state and 
    //optimization parameters

    SimTK::Vector initial_udot(_nCoordinates);
    realizeAccelerationFromParameters(_state, _init_parameters);

    int i = 0;
    for (const Coordinate& coord : _model->updComponentList<Coordinate>()) {
        initial_udot(i) = coord.getAccelerationValue(_state);
        i++;
    }

    i = 0;
    int j = 0;
    for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
        if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
            _constraint_desired_udot[i] = _observed_udot[j];
            _constraint_initial_udot[i] = initial_udot[j];
            i++;
        }
        if (_secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
            _constraint_desired_udot[i] = 0;
            _constraint_initial_udot[i] = initial_udot[j];
            i++;
        }
        j++;
    }

    //Calculate Unit Udots (change in accelerations due to unit change in 
    // optimization parameters)
    // rows - coordinates, columns - actuators

    _msl_unit_udot.resize(_nConstraints, _nMuscles);
    _non_muscle_actuator_unit_udot.resize(_nConstraints, _nNonMuscleActuators);
    _secondary_coord_unit_udot.resize(_nConstraints, _nSecondaryCoord);
    _secondary_speed_unit_udot.resize(_nConstraints, _nSecondaryCoord);
    _secondary_coord_unit_energy.resize(_nSecondaryCoord);
       
    _msl_unit_udot = -1;
    _non_muscle_actuator_unit_udot = -1;
    _secondary_coord_unit_udot = -1;
    _secondary_speed_unit_udot = -1;
    _secondary_coord_unit_energy = -1;

    //Compute initial Contact Energy
    _initial_contact_energy = 0;
    for (const auto& cnt_frc : 
        _model->updComponentList<Smith2018ArticularContactForce>()) {
        
        _initial_contact_energy += 
            cnt_frc.getOutputValue<double>(_state,"potential_energy");
    }

    //Compute Muscle Unit Udot
    j = 0;
    for (int i = 0; i < _nMuscles; ++i) {
        Muscle &msl = _model->updComponent<Muscle>(_muscle_path[i]);

        double force = _init_parameters[j] * _optimal_force[j];

        msl.setOverrideActuation(_state, force + 1.0);

        _model->realizeAcceleration(_state);
        
        int k = 0;
        for (const auto& coord : _model->updComponentList<Coordinate>()) {
            bool is_primary =
                _primary_coords.findIndex(coord.getAbsolutePathString()) > -1;
            bool is_secondary =
                _secondary_coords.findIndex(coord.getAbsolutePathString())> -1;

            if (is_primary || is_secondary) {

                _msl_unit_udot(k, j) = coord.getAccelerationValue(_state) - 
                    _constraint_initial_udot(k);
                k++;
            }
        }
        msl.setOverrideActuation(_state, force);
        j++;
    }

    //Compute Non Muscle Actuator Unit Udot
    j = 0;
    for (int i = 0; i < _nNonMuscleActuators; ++i) {
        ScalarActuator &actuator = 
            _model->updComponent<ScalarActuator>(_non_muscle_actuator_path[i]);

        double force = 
            _init_parameters[_nMuscles + j] * _optimal_force[_nMuscles + j];

        actuator.setOverrideActuation(_state, force + 1.0);

        _model->realizeAcceleration(_state);
        
        int k = 0;
         for (const auto& coord : _model->updComponentList<Coordinate>()) {
            bool is_primary =
                _primary_coords.findIndex(coord.getAbsolutePathString()) > -1;
            bool is_secondary =
                _secondary_coords.findIndex(coord.getAbsolutePathString())> -1;

            if (is_primary || is_secondary) {

                _non_muscle_actuator_unit_udot(k, j) =  
                    (coord.getAccelerationValue(_state) - 
                        _constraint_initial_udot(k));
                k++;
            }
        }
        actuator.setOverrideActuation(_state, force);

        j++;
    }

    //Compute Secondary Coordinate Unit Udot
    for (int j = 0; j < _nSecondaryCoord; ++j) {
        Coordinate& optim_coord = 
            _model->updComponent<Coordinate>(_secondary_coords[j]);

        double init_value = optim_coord.getValue(_state);

        double value = init_value + _unit_udot_epsilon;

        optim_coord.setValue(_state, value,true);
        
        _model->realizeAcceleration(_state);
        
        int k = 0;
        for (const auto& coord : _model->updComponentList<Coordinate>()) {
            bool is_primary =
                _primary_coords.findIndex(coord.getAbsolutePathString()) > -1;
            bool is_secondary =
                _secondary_coords.findIndex(coord.getAbsolutePathString())> -1;

            if (is_primary || is_secondary) {
                _secondary_coord_unit_udot(k, j) = _scale_delta_coord *
                    (coord.getAccelerationValue(_state) - 
                        _constraint_initial_udot(k)) / _unit_udot_epsilon;
                k++;
            }
        }
        
        //Contact Energy dot
        double cnt_energy = 0;
        for (Smith2018ArticularContactForce& cnt_frc : 
            _model->updComponentList<Smith2018ArticularContactForce>()) {
            cnt_energy += 
                cnt_frc.getOutputValue<double>(_state,"potential_energy"); 
        }
        _secondary_coord_unit_energy(j) = _scale_delta_coord *
            (cnt_energy - _initial_contact_energy)/_unit_udot_epsilon;

        optim_coord.setValue(_state, init_value, true);
    }
    _model->realizeAcceleration(_state);

    //Compute COMAK damping unit udot
    //Force in each damping actuator is zeroed in initialize()

    for (int j = 0; j < _nSecondaryCoord; ++j) {
        Coordinate& optim_coord = 
            _model->updComponent<Coordinate>(_secondary_coords[j]);

        double init_speed = optim_coord.getSpeedValue(_state);
        double speed_epsilon = _unit_udot_epsilon / _dt;

        optim_coord.setSpeedValue(_state, init_speed + speed_epsilon);

        _model->realizeAcceleration(_state);

        int k = 0;
        for (const auto& coord : _model->updComponentList<Coordinate>()) {
            bool is_primary =
                _primary_coords.findIndex(coord.getAbsolutePathString()) > -1;
            bool is_secondary =
                _secondary_coords.findIndex(coord.getAbsolutePathString())> -1;

            if (is_primary || is_secondary) {

                _secondary_speed_unit_udot(k, j) = _scale_delta_coord *
                    (coord.getAccelerationValue(_state) - 
                        _constraint_initial_udot(k)) / _unit_udot_epsilon;

                k++;
            }
        }

        optim_coord.setSpeedValue(_state, init_speed);
    }

    //Assemble Constraint Matrix 
    //Constraints = Rows
    //Optimized Parameters = Columns
    
    _constraint_matrix.resize(_nConstraints,_nParameters);
    _constraint_matrix = 0.0;

    for (int i = 0; i < _nConstraints; ++i) {
        int p = 0;

        //Muscles
        for (int j = 0; j < _nMuscles; ++j) {
            _constraint_matrix(i,p) = _optimal_force[p] * _msl_unit_udot(i, j);
            p++;
        }

        //Non Muscle Actuators
        for (int j = 0; j < _nNonMuscleActuators; ++j) {
            _constraint_matrix(i,p) = 
                _optimal_force[p] * _non_muscle_actuator_unit_udot(i, j);
            p++;
        }

        //Secondary Coordinates
        for (int j = 0; j < _nSecondaryCoord; ++j) {
            _constraint_matrix(i,p) = _secondary_coord_unit_udot(i,j);
            _constraint_matrix(i,p) += _secondary_speed_unit_udot(i,j);
            p++;
        }
    }

    if (_verbose > 5){
        std::cout << "Constraint Matrix" << std::endl;
        for (int i = 0; i < _nConstraints; ++i) {
            std::cout << _constraint_names[i] << std::endl;
            int p = 0;
                
            //Muscles
            std::cout << "Muscles" << std::endl;
            for (int j = 0; j < _nMuscles; ++j) {
                std::cout << _constraint_matrix(i, p) << std::endl;
                p++;
            }

            //Non Muscle Actuators
            std::cout << "Non Muscle Actuators" << std::endl;
            for (int j = 0; j < _nNonMuscleActuators; ++j) {
                std::cout << _constraint_matrix(i, p) << std::endl;
                p++;
            }

            //Secondary Coordinates
            std::cout << "Delta Secondary Coord" << std::endl;
            for (int j = 0; j < _nSecondaryCoord; ++j) {
                std::cout << _constraint_matrix(i, p) << std::endl;
                p++;
            }
        }
    }
}

void ComakTarget::realizeAccelerationFromParameters
    (SimTK::State& s, const SimTK::Vector &parameters) 
{
    //Apply Muscle Forces
    int j = 0;
    for (int i = 0; i < _nMuscles; ++i) {
        Muscle &msl = _model->updComponent<Muscle>(_muscle_path[i]);
        msl.overrideActuation(s, true);
        double force = _optimal_force[j] * parameters[j];
        msl.setOverrideActuation(s,force);
        j++;
    }

    //Apply Non Muscle Actuator Forces
    for (int i = 0; i < _nNonMuscleActuators; ++i) {
        ScalarActuator &actuator = 
            _model->updComponent<ScalarActuator>(_non_muscle_actuator_path[i]);
        actuator.overrideActuation(s, true);
        double force = _optimal_force[j] * parameters[j];
        actuator.setOverrideActuation(s,force);
        j++;
    }

    //Set Secondary Coordinates 
    for (int i = 0; i < _nSecondaryCoord; ++i) {
        Coordinate& coord = 
            _model->updComponent<Coordinate>(_secondary_coords[i]);

        double value = coord.getValue(s) + parameters(_nActuators + i);

        coord.setValue(s, value, false);
    }

    _model->assemble(s);
    _model->realizeAcceleration(s);
}

//=============================================================================
// OPTIMIZATION FUNCTIONS
//=============================================================================

//-----------------------------------------------------------------------------
// Objective Function (cost)
//-----------------------------------------------------------------------------

int ComakTarget::
objectiveFunc(const SimTK::Vector &parameters, const bool new_parameters,
    SimTK::Real &performance) const
{
    int p = 0;

    //Muscle Activations
    double msl_cost = 0.0;
    for (int i = 0; i < _nMuscles; i++) {
        msl_cost += _muscle_volumes(i) * _muscle_weight(i) *
            pow(fabs(parameters(p) - _desired_act(p)), _activation_exponent);
        p++;
    }
    
    //Non Muscle Actuators
    double non_muscle_cost = 0.0;
    for (int i = 0; i < _nNonMuscleActuators; i++) {
        non_muscle_cost +=  
            pow(fabs(parameters(p)), _activation_exponent);
        p++;
    }

    //Contact Energy
    //double contact_cost = 0.0;
    double contact_cost = _initial_contact_energy;
    for (int i = 0; i < _nSecondaryCoord; ++i) {
        contact_cost += parameters(p) * _secondary_coord_unit_energy[i];
        p++;
    }

    performance = msl_cost 
        + non_muscle_cost * _non_muscle_actuator_weight
        + contact_cost * _contact_energy_weight;

    if (_verbose > 9) {
        std::cout << "Total Cost: " << performance << "\t";
        std::cout << "Muscle Cost: " << msl_cost << "\t";
        std::cout << "Non Muscle Actuator Cost: " << non_muscle_cost << "\t";
        std::cout << "Contact Cost: " << contact_cost << std::endl;
    }

    return 0;
}

//-----------------------------------------------------------------------------
// Objective Function Gradient (w.r.t. optimization parameters)
//-----------------------------------------------------------------------------
int ComakTarget::
gradientFunc(const SimTK::Vector &parameters, const bool new_parameters,
    SimTK::Vector &gradient) const
{
    gradient = 0;
    int p = 0;
    for (int i = 0; i < _nMuscles; i++) {
        gradient[p] += _activation_exponent * 
            _muscle_volumes(i) * _muscle_weight(i) *
            (parameters[p] - _desired_act(p)) * 
            pow(fabs(_desired_act(p) - parameters[p]),
                _activation_exponent - 2.0);

        p++;
    }
    
    for (int i = 0; i < _nNonMuscleActuators; i++) {
        gradient[p] += 
            _activation_exponent * _non_muscle_actuator_weight * 
            parameters[p] * pow(fabs(parameters[p]),
                _activation_exponent - 2.0);

        p++;
    }

    /*
    int p = 0;
    for (int i = 0; i < _nMuscles; i++) {
        if (parameters[p]-_desired_act(i) < 0) {
            gradient[p] += -1.0 * _activation_exponent * _muscle_volumes(i) * 
                _muscle_weight(i) * pow(fabs(parameters[p] - _desired_act(p)), 
                    _activation_exponent - 1.0);
        }
        else {
            gradient[p] += _activation_exponent * _muscle_volumes(i) * 
                _muscle_weight(i) * pow(fabs(parameters[p] - _desired_act(p)),
                    _activation_exponent - 1.0);
        }
        p++;
    }
    
    for (int i = 0; i < _nNonMuscleActuators; i++) {
        if (parameters[p] < 0) {
            gradient[p] += 
                -1.0 * _activation_exponent * _non_muscle_actuator_weight * 
                pow(fabs(parameters[p]), 
                    _activation_exponent - 1.0);
        }
        else {
            gradient[p] += 
                _activation_exponent * _non_muscle_actuator_weight * 
                pow(fabs(parameters[p]),
                    _activation_exponent - 1.0);
        }
        
        p++;
    }
    */
    for (int i = 0; i < _nSecondaryCoord; ++i) {
        gradient[p] += 
            _contact_energy_weight * _secondary_coord_unit_energy[i];
        p++;
    }

    if (_verbose > 9) {
        std::cout << "Cost Gradient" << std::endl;
        for (int i = 0; i < _nParameters; ++i) {
            std::cout << _parameter_names[i] << " " << gradient[i]<< std::endl;
        }
    }
    return 0;
}

//-----------------------------------------------------------------------------
// Constraint Function 
//-----------------------------------------------------------------------------

int ComakTarget::
constraintFunc(const SimTK::Vector &parameters, bool new_parameters, 
    SimTK::Vector &constraints) const
{
    constraints = _constraint_initial_udot - _constraint_desired_udot + 
        _constraint_matrix * (parameters - _init_parameters);
 
    return 0;
}

//-----------------------------------------------------------------------------
// Constraint Jacobian
// (gradient of contstraints w.r.t. optimization parameters)
//-----------------------------------------------------------------------------
int ComakTarget::constraintJacobian(const SimTK::Vector &parameters,
    bool new_parameters, SimTK::Matrix &jac) const
{
    jac = _constraint_matrix;

    return 0;
}

void ComakTarget::setParameterBounds(double scale) {

    SimTK::Vector lower_bounds(_nParameters), upper_bounds(_nParameters);
    int p = 0;

    //Muscles 
    for (int i = 0; i < _nMuscles; ++i) {
        Muscle &msl = _model->updComponent<Muscle>(_muscle_path[i]);

        double min_value = msl.getMinControl();
        double max_value = msl.getMaxControl();

        lower_bounds(p) = min_value;
        upper_bounds(p) = max_value;
        p++;
    }

    //Non Muscle Actuators
    for (int i = 0; i < _nNonMuscleActuators; ++i) {
        ScalarActuator &actuator = 
            _model->updComponent<ScalarActuator>(_non_muscle_actuator_path[i]);

        double min_control = actuator.getMinControl();
        double max_control = actuator.getMaxControl();
        double min_value;
        double max_value;
        if (min_control > -1000)
            min_value = actuator.getMinControl()*scale;
        else {
            min_value = -1000*scale;
        }
        if (max_control < 1000)
            max_value = actuator.getMinControl()*scale;
        else {
            max_value = 1000*scale;
        }
        
        lower_bounds(p) = min_value;
        upper_bounds(p) = max_value;
        p++;
    }

    //Secondary Coordinates
    for (int j = 0; j < _nSecondaryCoord; j++) {
        lower_bounds(p) = - _max_change[j] * scale / _scale_delta_coord;
        upper_bounds(p) = _max_change[j] * scale / _scale_delta_coord;
        p++;
    }
    setParameterLimits(lower_bounds, upper_bounds);

    if (_verbose > 5) {
        std::cout 
            << std::setw(20) << "Parameter Limits: "
            << std::setw(15) << "initial value"
            << std::setw(15) << "lower bound"
            << std::setw(15) << "upper bound"
            << std::endl;
        std::cout << "------------------------------------------" << std::endl;

        for (int i = 0; i < _nParameters; ++i) {
            std::cout << i << " "
                << std::setw(17)  << _parameter_names[i] 
                << std::setw(15)  <<_init_parameters[i] 
                << std::setw(15)  << lower_bounds(i) 
                << std::setw(15)  << upper_bounds(i) << std::endl;
        }
    }
}

void ComakTarget::printPerformance(SimTK::Vector parameters) {
    bool notNeeded = false;
    SimTK::Real performance;
    objectiveFunc(parameters, notNeeded, performance);

    std::cout << "\nOptimization Cost Function: " << performance << std::endl;
}

