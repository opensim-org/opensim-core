#ifndef OPENSIM_MUSCLEACTIVATIONDYNAMICS_H_
#define OPENSIM_MUSCLEACTIVATIONDYNAMICS_H_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  MuscleActivationDynamics.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Thomas Uchida, Ajay Seth                                        *
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

#include "Simbody.h"
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Common/Object.h>

namespace OpenSim {
/** A base class for modeling muscle activation dynamics. This class implements
    a zeroth-order model, simply setting activation to the default value.
    Activation models that derive from this base class must override the
    getActivation() virtual method.

    <b>Properties</b>
    \li \c minimum_activation: Smallest permitted activation value
    \li \c maximum_activation: Largest permitted activation value
    \li \c default_activation: Default activation value

    <b>Conditions</b>
    \verbatim
    0 <= minimum_activation <= default_activation <= maximum_activation <= 1
    \endverbatim

    <b>Default %Property Values</b>
    \verbatim
    minimum_activation .... 0
    maximum_activation .... 1
    default_activation .... 0.5
    \endverbatim

    @author Thomas Uchida, Ajay Seth
**/
class OSIMACTUATORS_API MuscleActivationDynamics : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleActivationDynamics, Object);
public:

//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
        These are the serializable properties associated with this class. **/
    //@{
    OpenSim_DECLARE_PROPERTY(minimum_activation, double,
        "Smallest permitted activation value");
    OpenSim_DECLARE_PROPERTY(maximum_activation, double,
        "Largest permitted activation value");
    OpenSim_DECLARE_PROPERTY(default_activation, double,
        "Default activation value");
    //@}

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** @name Constructors **/
    //@{

    /** Default constructor. Creates an activation dynamic model with the
        default property values and assigns it a default name. **/
    MuscleActivationDynamics();

    /** Creates an activation dynamic model using the provided properties. **/
    MuscleActivationDynamics(double minimumActivation,
                             double maximumActivation,
                             double defaultActivation,
                             const std::string& name);
    //@}

    //--------------------------------------------------------------------------
    // ACCESSORS AND MUTATORS
    //--------------------------------------------------------------------------
    /** @name Accessors and Mutators **/
    //@{

    /** Get/set the smallest permitted activation value. **/
    double getMinimumActivation() const;
    void setMinimumActivation(double minimumActivation);

    /** Get/set the largest permitted activation value. **/
    double getMaximumActivation() const;
    void setMaximumActivation(double maximumActivation);

    /** Get/set the default activation value. **/
    double getDefaultActivation() const;
    void setDefaultActivation(double defaultActivation);

    //@}

    //--------------------------------------------------------------------------
    // OTHER PUBLIC METHODS
    //--------------------------------------------------------------------------
    /** Returns the current activation level. Activation models that derive from
        this base class must override the getActivation() virtual method. **/
    virtual double getActivation(const SimTK::State& s) const;

//==============================================================================
// PRIVATE METHODS
//==============================================================================
private:
    void setNull();
    void constructProperties();

}; // end of class MuscleActivationDynamics
}  // end of namespace OpenSim

#endif //OPENSIM_MUSCLEACTIVATIONDYNAMICS_H_
