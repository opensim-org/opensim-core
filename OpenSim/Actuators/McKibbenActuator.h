#ifndef OPENSIM_MCKIBBEN_ACTUATOR_H_
#define OPENSIM_MCKIBBEN_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  McKibbenActuator.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Nabeel Allana                                                   *
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

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/PathActuator.h>

namespace OpenSim
{

/**
* McKibben Pneumatic Actuator Model based on the simple cylindrical
* formulation described in J. Dyn. Sys., Meas., Control 122, 386-388 
* (1998) (3 pages); doi:10.1115/1.482478.
*
* Pressure is used as a control signal. There is an optional 'cord'
* attached to the actuator which allows for the path length of the actuator
* to be shorter than the total distance spanned by the points to which the
* actuator is connected. By default its length is zero. Please
* refer to the above paper for details regarding the rest of the
* properties.
*
* @author Nabeel Allana
*/

class OSIMACTUATORS_API McKibbenActuator : public PathActuator
{
OpenSim_DECLARE_CONCRETE_OBJECT(McKibbenActuator, PathActuator);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(thread_length, double,
        "The thread length");
    OpenSim_DECLARE_PROPERTY(number_of_turns, double,
        "Number of turns of the thread.");
    OpenSim_DECLARE_PROPERTY(cord_length, double,
        "The length of the flexible cord attaching the actuator to the last point.");
    
//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor leaves body names unspecified. **/
    McKibbenActuator();
    /** Convenience constructor for API users. **/
    McKibbenActuator(const std::string& name, double num_turns, double thread_length);

    /** %Set the 'number of turns' property. **/
    void setNumberOfTurns(double val)
    {   set_number_of_turns(val); }
    /** Get the current value of the 'number of turns' property. **/
    double getNumberOfTurns() const 
    {   return get_number_of_turns(); }
    
    /** %Set the 'thread length' property. **/
    void setThreadLength(double val)
    {   set_thread_length(val); }
    /** Get the current value of the 'thread length' property. **/
    double getThreadLength() const
    {   return get_thread_length(); }
    
    /** %Set the 'cord length' property. **/
    void setCordLength(double val)
    {   set_cord_length(val); }
    /** Get the current value of the 'cord length' property. **/
    double getCordLength() const
    {   return get_cord_length(); }

    /** Compute actuation for current state. **/
    double computeActuation(const SimTK::State& s) const override;
protected:
    /** how to display the McKibben
    VisibleObject _displayer; */

private:
    void constructProperties();

    //--------------------------------------------------------------------------
    // Implement Force interface
    //--------------------------------------------------------------------------
    void computeForce(const SimTK::State& state, 
                      SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                      SimTK::Vector& mobilityForces) const override;


    //--------------------------------------------------------------------------
    // Implement ModelComponent interface
    //--------------------------------------------------------------------------
    // Setup method initializes Body reference pointers to match the names.
    void extendConnectToModel(Model& aModel) override;

    //--------------------------------------------------------------------------
    // Visualization interface
    //--------------------------------------------------------------------------


}; // class McKibbenActuator
} // namespace OpenSim

#endif // OPENSIM_MCKIBBEN_ACTUATOR_H_
