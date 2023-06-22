#ifndef OPENSIM_CONTROLLER_H_
#define OPENSIM_CONTROLLER_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Controller.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Frank C. Anderson, Chand T. John, Samuel R. Hamner   *
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

//============================================================================
// INCLUDE
//============================================================================
// These files contain declarations and definitions of variables and methods
// that will be used by the Controller class.
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/Set.h>

namespace OpenSim { 

// Forward declarations of classes that are used by the controller implementation
class Model;
class Actuator;

/**
 * Controller is an abstract ModelComponent that defines the interface for   
 * an OpenSim Controller. A controller computes and sets the values of the  
 * controls for the actuators under its control.
 * The defining method of a Controller is its computeControls() method.
 * @see computeControls()
 *
 * @note Controllers currently do not use the Socket mechanism to locate 
 * and connect to the Actuators that Controllers depend on. As a result,
 * for now, Controllers do not support controlling multiple actuators with 
 * the same name.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Controller : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Controller, ModelComponent);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** Controller is enabled (active) by default.
    NOTE: Prior to OpenSim 4.0, this property was named **isDisabled**.
          If **isDisabled** is **true**, **enabled** is **false**.
          If **isDisabled** is **false**, **enabled** is **true**.            */
    OpenSim_DECLARE_PROPERTY(enabled, bool, 
        "Flag (true or false) indicating whether or not the controller is "
        "enabled." );

    OpenSim_DECLARE_LIST_PROPERTY(actuator_list, std::string,
        "The list of model actuators that this controller will control."
        "The keyword ALL indicates the controller will control all the "
        "actuators in the model" );

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
public:

    /** Default constructor. */
    Controller();
    Controller(Controller const&);
    Controller& operator=(Controller const&);
    ~Controller() noexcept override;

    // Uses default (compiler-generated) destructor, copy constructor and copy 
    // assignment operator.

    //--------------------------------------------------------------------------
    // Controller Interface
    //--------------------------------------------------------------------------
    /** Get whether or not this controller is enabled.
     * @return true when controller is enabled.
     */
    bool isEnabled() const;

    /** Enable this controller.
     * @param enableFlag Enable the controller if true.
     */
    void setEnabled(bool enableFlag);

    /** replace the current set of actuators with the provided set */
    void setActuators(const Set<Actuator>& actuators );
    /** add to the current set of actuators */
    void addActuator(const Actuator& actuator);
    /** get a const reference to the current set of const actuators */
    const Set<const Actuator>& getActuatorSet() const;
    /** get a writable reference to the set of const actuators for this controller */
    Set<const Actuator>& updActuators();

    /** Compute the control for actuator
     *  This method defines the behavior for any concrete controller 
     *  and therefore must be implemented by concrete subclasses.
     *
     * @param s         system state 
     * @param controls  writable model controls (all actuators)
     */
    virtual void computeControls(const SimTK::State& s,
                                 SimTK::Vector &controls) const = 0;

    int getNumControls() const {return _numControls;}

protected:

    /** Model component interface that permits the controller to be "wired" up
       to its actuators. Subclasses can override to perform additional setup. */
    void extendConnectToModel(Model& model) override;  

    /** Model component interface that creates underlying computational components
        in the SimTK::MultibodySystem. This includes adding states, creating 
        measures, etc... required by the controller. */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    /** Only a Controller can set its number of controls based on its actuators */
    void setNumControls(int numControls) {_numControls = numControls; }

    void updateFromXMLNode(SimTK::Xml::Element& node,
                           int versionNumber) override;

private:
    // number of controls this controller computes 
    int _numControls;

    // the (sub)set of Model actuators that this controller controls */ 
    Set<const Actuator> _actuatorSet;

    // construct and initialize properties
    void constructProperties();

//=============================================================================
};  // END of class Controller

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_CONTROLLER_H_


