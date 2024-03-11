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

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/Set.h>

namespace OpenSim { 

class Model;
class Actuator;

/**
 * Controller is an abstract ModelComponent that defines the interface for   
 * an OpenSim Controller. A controller computes and sets the values of the  
 * controls for the actuators under its control.
 *
 * The defining method of a Controller is its computeControls() method. All
 * concrete controllers must implement this method.
 * @see computeControls()
 *
 * Actuators can be connected to a Controller via the list Socket `actuators`.
 * Connection can be made via the `addActuator()` convenience method or through
 * the Socket directly:
 *
 * @code{.cpp}
 * // Add an actuator to the controller.
 * const auto& actuator = model.getComponent<Actuator>("/path/to/actuator");
 * controller.addActuator(actuator);
 *
 * // Connect an actuator to the controller via the actuators Socket.
 * controller.appendSocketConnectee_actuators(actuator);
 * @endcode
 *
 * Multiple actuators can be connected to a Controller via the `setActuators()`
 * convenience methods:
 *
 * @code{.cpp}
 * // Add a Model's Set of Actuators to the controller.
 * controller.setActuators(model.getActuators());
 *
 * // Add a ComponentList of Actuators to the controller.
 * controller.setActuators(model.getComponentList<Actuator>());
 * @endcode
 *
 * @note Prior to OpenSim 4.6, controlled actuators were managed via the list
 *       Property `actuator_list`. This interface is no longer supported, all
 *       actuators must be connected via the `actuators` list Socket.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Controller : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Controller, ModelComponent);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    /** Controller is enabled (active) by default.
    NOTE: Prior to OpenSim 4.0, this property was named **isDisabled**.
          If **isDisabled** is **true**, **enabled** is **false**.
          If **isDisabled** is **false**, **enabled** is **true**. */
    OpenSim_DECLARE_PROPERTY(enabled, bool, 
        "Flag (true or false) indicating whether or not the controller is "
        "enabled." );

    OpenSim_DECLARE_LIST_SOCKET(actuators, Actuator,
        "The list of Actuators that this controller will control.");

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
    Controller();
    ~Controller() noexcept override;

    Controller(const Controller&);
    Controller& operator=(Controller const&);

    Controller(Controller&&);
    Controller& operator=(Controller&&);

    //--------------------------------------------------------------------------
    // CONTROLLER INTERFACE
    //--------------------------------------------------------------------------
    /** Get whether or not this controller is enabled.
     * @return true when controller is enabled.
     */
    bool isEnabled() const;

    /** Enable this controller.
     * @param enableFlag Enable the controller if true.
     */
    void setEnabled(bool enableFlag);

    /** Replace the current set of actuators with the provided set. */
    void setActuators(const Set<Actuator>& actuators);
    void setActuators(const ComponentList<const Actuator>& actuators);

    /** Add to the current set of actuators. */
    void addActuator(const Actuator& actuator);

    /** Compute the control for actuator
     *  This method defines the behavior for any concrete controller 
     *  and therefore must be implemented by concrete subclasses.
     *
     * @param s         system state 
     * @param controls  writable model controls (all actuators)
     */
    virtual void computeControls(const SimTK::State& s,
                                 SimTK::Vector &controls) const = 0;

    /** Get the number of controls this controller computes. */
    int getNumControls() const { return _numControls; }

    /** Get the number of actuators that this controller is connected to. */
    int getNumActuators() const {
        return static_cast<int>(
            getSocket<Actuator>("actuators").getNumConnectees());
    }

protected:
    /** Only a Controller can set its number of controls based on its
     * actuators. */
    void setNumControls(int numControls) { _numControls = numControls; }

    // MODEL COMPONENT INTERFACE
    void updateFromXMLNode(SimTK::Xml::Element& node,
                           int versionNumber) override;
    void extendConnectToModel(Model& model) override;

private:
    // The number of controls this controller computes.
    int _numControls;

    // Construct and initialize properties.
    void constructProperties();

    // Used to temporarily store actuator names when reading from XML prior to
    // XMLDocument version 40600. This is used to support backwards
    // compatibility with models that use the old actuator_list property. The
    // actuator names are used to find actuators in the model and connect them
    // to the list Socket.
    std::vector<std::string> _actuatorNamesFromXML;

};  // class Controller

} // namespace OpenSim

#endif // OPENSIM_CONTROLLER_H_


