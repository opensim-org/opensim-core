#ifndef OPENSIM_MUSCLE_ACTIVATION_DYNAMICS_H_
#define OPENSIM_MUSCLE_ACTIVATION_DYNAMICS_H_
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
 * Author(s): Thomas Uchida, Ajay Seth, Michael Sherman                       *
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
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
/** An abstract class for modeling muscle activation dynamics. Activation models
    that derive from this base class are responsible for creating and
    maintaining whatever states and/or cache variables they require to override
    the getActivation() and setActivation() pure virtual methods.

    <b>Properties</b>
    \li \c minimum_activation: Smallest permitted activation value.
    \li \c maximum_activation: Largest permitted activation value.
    \li \c default_activation: Default activation value.

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

    @author Thomas Uchida, Ajay Seth, Michael Sherman
**/

class OSIMACTUATORS_API MuscleActivationDynamics : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(MuscleActivationDynamics, ModelComponent);
public:
    /** The %ExcitationGetter abstract class defines a standard interface for
        supplying muscle excitation to activation models. Each muscle must
        provide an implementation of this class that overrides the pure virtual
        getExcitation() method. **/
    class ExcitationGetter {
    public:
        virtual ~ExcitationGetter() {}
        virtual double getExcitation(const SimTK::State& s) const = 0;
    };

//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property Declarations
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
    /** @name Constructors and Destructor **/
    //@{

    /** Creates an activation dynamic model with the default property values and
        assigns it a default name. An %ExcitationGetter must be created for
        obtaining muscle excitation. **/
    MuscleActivationDynamics();

    /** Creates an activation dynamic model with the default property values,
        the specified name, and the specified %ExcitationGetter. Takes ownership
        of the %ExcitationGetter object. **/
    MuscleActivationDynamics(const std::string& name,
                             ExcitationGetter* getter);

    /** Deletes the %ExcitationGetter object. **/
    ~MuscleActivationDynamics();

    //@}

    //--------------------------------------------------------------------------
    // ACCESSORS AND MUTATORS
    //--------------------------------------------------------------------------
    /** @name Accessors and Mutators **/
    //@{

    /** Get the smallest permitted activation value. **/
    double getMinimumActivation() const;
    /** %Set the smallest permitted activation value. Clamps minimum_activation
        to the interval [0, maximum_activation] and default_activation to the
        interval [minimum_activation, maximum_activation]. **/
    void setMinimumActivation(double minimumActivation);

    /** Get the largest permitted activation value. **/
    double getMaximumActivation() const;
    /** %Set the largest permitted activation value. Clamps maximum_activation
        to the interval [minimum_activation, 1] and default_activation to the
        interval [minimum_activation, maximum_activation]. **/
    void setMaximumActivation(double maximumActivation);

    /** Get the default activation value. **/
    double getDefaultActivation() const;
    /** %Set the default activation value. Clamps default_activation to the
        interval [minimum_activation, maximum_activation]. **/
    void setDefaultActivation(double defaultActivation);

    /** Define an %ExcitationGetter for obtaining muscle excitation. Takes
        ownership of the %ExcitationGetter object. **/
    void setExcitationGetter(ExcitationGetter* getter);

    //@}

    //--------------------------------------------------------------------------
    // STATE-DEPENDENT METHODS
    //--------------------------------------------------------------------------
    /** Get the current activation. **/
    virtual double getActivation(const SimTK::State& s) const = 0;

    /** %Set activation to the value provided. **/
    virtual void setActivation(SimTK::State& s, double activation) const = 0;

    /** Get the muscle excitation using the %ExcitationGetter object. Returns
        zero if no %ExcitationGetter exists. **/
    double getExcitation(const SimTK::State& s) const;

//==============================================================================
// PROTECTED METHODS
//==============================================================================
protected:
    /** Clamp to the interval [minimum_activation, maximum_activation]. **/
    double clampToValidInterval(double val) const;

//==============================================================================
// PRIVATE METHODS
//==============================================================================
private:
    void setNull();
    void constructProperties();

    ExcitationGetter* _excitationGetter;

}; // end of class MuscleActivationDynamics
}  // end of namespace OpenSim

#endif //OPENSIM_MUSCLE_ACTIVATION_DYNAMICS_H_
