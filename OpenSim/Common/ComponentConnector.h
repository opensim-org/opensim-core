#ifndef OPENSIM_COMPONENT_CONNECTOR_H_
#define OPENSIM_COMPONENT_CONNECTOR_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Connector.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                           *
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

/** @file
 * This file defines the Connector class, which formalizes the dependency of 
 * of a Component on another Object/Component in order to operate, BUT it does 
 * not own it. While Components can be composites (of multiple components) 
 * they often depend on unrelated objects/components that are define elsewhere.
 * For example a Joint connects two bodies together, neither or at best the
 * child body can own the joint. It must have a "Connector" to a parent body
 * that already exists. The maintenance of the dependency and the run-time 
 * verification of the existance of the parent is the task of the Connector.
 */

// INCLUDES
#include "OpenSim/Common/Object.h"
#include "OpenSim/Common/ComponentOutput.h"

namespace OpenSim {

//=============================================================================
//                        OPENSIM COMPONENT CONNECTOR
//=============================================================================
/**
 * A Connector formalizes the need for a connection between a Component and a
 * dependent object, without owning the object it is connected to. The purpose
 * of a Connector is to specify: 1) the object type that the Component is 
 * dependent on, 2) by when (what stage) the connector must be connected in
 * order for the component to function, 3) the name of object to connect to and
 * 4) whether it is connected or not. The specific instance that satisfies the
 * connector's requirement is maintained by the Component's list of Connections.
 *
 * For example, a Joint has one Connector for the parent Body that it joins
 * its owning Body to. The type for the connector is Body and any attempt to 
 * connect to a non-Body object will throw an exception.
 * The connectAt Stage is Topology. That is the Joint's connection to a Body 
 * must be performed at the Topology system stage, and any attempt to change 
 * the connection status will invalidate that Stage and above.
 *
 * Other Components like a Marker or a Probe that do not change the system
 * topology or add new states could potentially be connected at later stages
 * like Model or Instance.
 *
 * @author  Ajay Seth
 */

class OSIMCOMMON_API AbstractConnector : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(AbstractConnector, Object);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with a Connector. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(connected_to_name, std::string,
        "Name of the component this Connector should be connected to.");
    /**@}**/
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /** Default constructor */
    AbstractConnector() : Object(), connectAtStage(SimTK::Stage::Topology) {
        constructProperties();
    }

    // default destructor, copy constructor

    /** Convenience constructor 
        Create a Connector with specified name and stage at which it
        should be connected.
    @param name             name of the connector, usually desribes its dependency. 
    @param connectAtStage   Stage at which Connector should be connected. */
    AbstractConnector(const std::string& name, const SimTK::Stage& connectAtStage) :
        connectAtStage(connectAtStage) {
        constructProperties();
        setName(name);
    }

    // default copy assignment

    /** get the system Stage when the connection should be made */
    SimTK::Stage getConnectAtStage() const {
        return connectAtStage;
    }

    virtual ~AbstractConnector() {};

    //--------------------------------------------------------------------------
    /** Derived classes must satisfy this Interface */
    //--------------------------------------------------------------------------
    /** Is the Connector connected to anything? */
    virtual bool isConnected() const = 0;

    /** get the type of object this connector connects to*/
    virtual std::string getConnectedToTypeName() const = 0;

    /** Connect this Connector to the provided connectee object */
    virtual void connect(const Object& connectee) = 0;

    /** Disconnect this Connector from the connectee object */
    virtual void disconnect() = 0;

private:
    void constructProperties() { constructProperty_connected_to_name(""); }
    SimTK::Stage connectAtStage;
//=============================================================================
};  // END class AbstractConnector


template<class T>
class  Connector : public AbstractConnector {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(Connector, T, AbstractConnector);
public:
    /** Default constructor */
    Connector() : connectee(nullptr) {}

    // default destructor, copy constructor

    /** Convenience constructor
    Create a Connector that can only connect to Object of type T with specified 
    name and stage at which it should be connected.
    @param name             name of the connector used to describe its dependency.
    @param connectAtStage   Stage at which Connector should be connected. */
    Connector(const std::string& name, const SimTK::Stage& connectAtStage) : 
        AbstractConnector(name, connectAtStage), connectee(nullptr) {}

    virtual ~Connector() {}

    /** Is the Connector connected to object of type T? */
    bool isConnected() const override {
        return !connectee.empty();
    }

    /** Temporary access to the connectee for testing purposes. Real useage
        will be through the Connector (and Input) interfaces. 
        For example, Input should short circuit to its Output's getValue()
        once it is connected.
    Return a const reference to the object connected to this Connector */
    const T& getConnectee() const { return connectee.getRef(); }

    /** Connect this Connector to the provided connectee object */
    void connect(const Object& object) override{
        const T* objT = dynamic_cast<const T*>(&object);
        if (objT) {
            connectee = *objT;
            set_connected_to_name(object.getName());
        }
        else {
            std::stringstream msg;
            msg << "Connector::connect(): ERR- Cannot connect '" << object.getName()
                << "' of type " << object.getConcreteClassName() << ". Connector requires "
                << getConnectedToTypeName() << ".";
            throw Exception(msg.str(), __FILE__, __LINE__);
        }
    }

    void disconnect() override {
        connectee.clear();
    }
    
    /** Derived classes must satisfy this Interface */
    /** get the type of object this connector connects to*/
    std::string getConnectedToTypeName() const override
    { return T::getClassName(); }

    SimTK_DOWNCAST(Connector, AbstractConnector);

private:
    mutable SimTK::ReferencePtr<const T> connectee;
}; // END class Connector<T>


/** A specialized Connector that connects to an Output signal is an Input.
    An AbstractInput enables maintenance of a list of unconnected Inputs. 
*/

class OSIMCOMMON_API AbstractInput : public AbstractConnector{
    OpenSim_DECLARE_ABSTRACT_OBJECT(AbstractInput, AbstractConnector);
public:
    /** Default constructor */
    AbstractInput() : AbstractConnector(), connectee(nullptr) {}
    /** Convenience constructor
    Create an AbstractInput (Connector) that connects only to an AbstractOutput
    specified by name and stage at which it should be connected.
    @param name             name of the dependent (Abstract)Output.
    @param connectAtStage   Stage at which Input should be connected. */
    AbstractInput(const std::string& name, const SimTK::Stage& connectAtStage) :
        AbstractConnector(name, connectAtStage), connectee(nullptr) {}

    virtual ~AbstractInput() {}

    // Connector interface
    void connect(const Object& object) override{
        std::stringstream msg;
        msg << "Input::connect(): ERR- Cannot connect '" << object.getName()
            << "' of type " << object.getConcreteClassName() <<
            ". Input can only connect to an Output.";
        throw Exception(msg.str(), __FILE__, __LINE__);
    }

    /** Input Specific Connect */
    virtual void connect(const AbstractOutput& output) const {
        connectee = output;
        //std::cout << getConcreteClassName() << "::connected to '";
        //std::cout << output.getName() << "'<" << output.getTypeName();
        //std::cout << ">." << std::endl;
    }

    void disconnect() override {
        connectee.clear();
    }

    /** Is the Input connected to an Output? */
    bool isConnected() const override {
        return !connectee.empty();
    }

    /** Derived classes must satisfy this Interface */
    /** get the type of object this connector connects to*/
    std::string getConnectedToTypeName() const override
    { return SimTK::NiceTypeName<AbstractOutput>::name(); }

private:
    mutable SimTK::ReferencePtr<const AbstractOutput> connectee;
//=============================================================================
};  // END class AbstractInput


/** An Input<T> must be connected by an Output<T> */
template<class T>
class  Input : public AbstractInput {
    OpenSim_DECLARE_CONCRETE_OBJECT(Input, AbstractInput);
public:
    /** Default constructor */
    Input() : AbstractInput() {}
    /** Convenience constructor
    Create an Input<T> (Connector) that can only connect to an Output<T>
    name and stage at which it should be connected.
    @param name             name of the Output dependency.
    @param connectAtStage   Stage at which Input should be connected. */
    Input(const std::string& name, const SimTK::Stage& connectAtStage) :
        AbstractInput(name, connectAtStage) {}

    /** Connect this Input the from provided (Abstract)Output */
    void connect(const AbstractOutput& output) const override{
        // enable interaction through AbstractInterface
        Super::connect(output);
        // and value specific interface
        connectee = Output<T>::downcast(output);
    }

    /**Get the value of this Input when it is connected. Redirects to connected
       Output<T>'s getValue() with minimal overhead. */
    const T& getValue(const SimTK::State &state) const {
        return connectee.getRef().getValue(state);
    }

    SimTK_DOWNCAST(Input, AbstractInput);

private:
    mutable SimTK::ReferencePtr< const Output<T>  > connectee;
}; // END class Input<T>


/** A SimTK::Measure_ whose value is the value of an OpenSim::Input, and whose
* dependsOn SimTK::Stage is the connectAt stage of the OpenSim::Input. This
* Measure is useful for building OpenSim Components that use
* a SimTK::Measure_ internally.
*/
template <class T>
class InputMeasure : public SimTK::Measure_<T> {
public:
    SimTK_MEASURE_HANDLE_PREAMBLE(InputMeasure, SimTK::Measure_<T>);

    InputMeasure(SimTK::Subsystem& sub, const OpenSim::Input<T>& input)
        :   SimTK::Measure_<T>(sub, new Implementation(input),
            SimTK::AbstractMeasure::SetHandle()) {}

    SimTK_MEASURE_HANDLE_POSTSCRIPT(InputMeasure, SimTK::Measure_<T>);
};

template <class T>
class InputMeasure<T>::Implementation :
        public SimTK::Measure_<T>::Implementation {
public:
    Implementation(const OpenSim::Input<T>& input)
        :   SimTK::Measure_<T>::Implementation(), m_input(input) {}

    Implementation* cloneVirtual() const override
    {return new Implementation(*this);}
    int getNumTimeDerivativesVirtual() const override {return 0;}
    SimTK::Stage getDependsOnStageVirtual(int order) const override
    { return m_input.getConnectAtStage(); }

    // The meat of this class: return the Input's value.
    void calcCachedValueVirtual(const SimTK::State& s,
            int derivOrder, T& value) const override
    {
        SimTK_ASSERT1_ALWAYS(derivOrder == 0,
                "InputMeasure::Implementation::calcCachedValueVirtual(): "
                        "derivOrder %d seen but only 0 allowed.", derivOrder);

        value = m_input.getValue(s);
    }

private:
    const OpenSim::Input<T>& m_input;
}; // END class InputMeasure<T>.

} // end of namespace OpenSim

#endif  // OPENSIM_COMPONENT_CONNECTOR_H_
