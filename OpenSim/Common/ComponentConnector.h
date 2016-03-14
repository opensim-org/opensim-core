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
 * Author(s): Ajay Seth                                                       *
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
 * they often depend on unrelated objects/components that are defined and
 * owned elsewhere.
 *
 * For example a Joint connects two bodies together, but the Joint does 
 * not own either body. Instead, the Joint has Connectors to a parent and 
 * a child body that already exists. The maintenance of the dependency and 
 * the run-time verification of the existence of the bodies is the duty
 * of the Connector.
 */

// INCLUDES
#include "OpenSim/Common/ComponentOutput.h"
#include "OpenSim/Common/ComponentList.h"
#include <Simbody.h>

namespace OpenSim {

//=============================================================================
//                        OPENSIM COMPONENT CONNECTOR
//=============================================================================
/**
 * A Connector formalizes the dependency between a Component and another object
 * (typically another Component) without owning that object. The object that
 * satisfies the requirements of the Connector we term the "connectee". When a 
 * Connector is satisfied by a connectee we have a successful "connection" or
 * is said to be connected.
 *
 * The purpose of a Connector is to specify: 1) the connectee type that the
 * Component is dependent on, 2) by when (what stage) the connector must be
 * connected in order for the component to function, 3) the name of a connectee
 * that can be found at run-time to satisfy the connector, and 4) whether or
 * not it is connected. A Connector maintains a reference to the instance
 * (connectee) until it is disconnected.
 *
 * For example, a Joint has two Connectors for the parent and child Bodies that
 * it joins. The type for the connector is a PhysicalFrame and any attempt to 
 * connect to a non-Body (or frame rigidly attached to a Body) will throw an
 * exception. The connectAt Stage is Topology. That is, the Joint's connection to
 * a Body must be performed at the Topology system stage, and any attempt to
 * change the connection status will invalidate that Stage and above.
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
    OpenSim_DECLARE_PROPERTY(connectee_name, std::string,
        "Name of the component this Connector should be connected to.");

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
    @param name             name of the connector, usually describes its dependency. 
    @param connectAtStage   Stage at which Connector should be connected.
    @param isList           Whether this Connector can have multiple connectees.
    @param owner            Component to which this Connector belongs.*/
    AbstractConnector(const std::string& name,
                      const SimTK::Stage& connectAtStage,
                      bool isList,
                      const Component& owner) :
                      _owner(&owner), connectAtStage(connectAtStage),
                      _isList(isList) {
        constructProperties();
        setName(name);
    }

    // default copy assignment

    virtual ~AbstractConnector() {};
    
    /** get the system Stage when the connection should be made */
    SimTK::Stage getConnectAtStage() const {
        return connectAtStage;
    }
    
    bool isListConnector() const { return _isList; }

    //--------------------------------------------------------------------------
    /** Derived classes must satisfy this Interface */
    //--------------------------------------------------------------------------
    /** Is the Connector connected to anything? */
    virtual bool isConnected() const = 0;
    
    /** The number of connectees connected to this connector. This is either
        0 or 1 for a non-list connector. */
    virtual size_t getNumConnectees() const = 0;

    /** Get the type of object this connector connects to*/
    virtual std::string getConnecteeTypeName() const = 0;

    /** Connect this Connector to the provided connectee object. If this is a
        list connector, the connectee is appended to the list of connectees;
        otherwise, the provided connectee replaces the single connectee. */
    virtual void connect(const Object& connectee) = 0;

    /** Connect this Connector according to its connectee_name property
        given a root Component to search its subcomponents for the connect_to
        Component. */
    virtual void findAndConnect(const Component& root) {
        throw Exception("findAndConnect() not implemented; not supported "
                        "for this type of connector", __FILE__, __LINE__);
    }

    /** Disconnect this Connector from its connectee. */
    virtual void disconnect() = 0;

protected:
    const Component& getOwner() const { return _owner.getRef(); }
    void setOwner(const Component& o) { _owner.reset(&o); }

private:
    void constructProperties() {
        constructProperty_connectee_name("");
    }
    SimTK::Stage connectAtStage;
    bool _isList = false;

    SimTK::ReferencePtr<const Component> _owner;

    friend Component;

//=============================================================================
};  // END class AbstractConnector


template<class T>
class  Connector : public AbstractConnector {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(Connector, T, AbstractConnector);
public:
    
    /** Default constructor */
    Connector() {}

    // default destructor, copy constructor

    /** Convenience constructor
    Create a Connector that can only connect to Object of type T with specified 
    name and stage at which it should be connected.
    @param name             name of the connector used to describe its dependency.
    @param connectAtStage   Stage at which Connector should be connected.
    @param owner The component that contains this input. */
    Connector(const std::string& name, const SimTK::Stage& connectAtStage,
              Component& owner) :
        AbstractConnector(name, connectAtStage, false, owner),
        connectee(nullptr) {}

    virtual ~Connector() {}

    /** Is the Connector connected to object of type T? */
    bool isConnected() const override {
        return !connectee.empty();
    }
    
    size_t getNumConnectees() const override {
        return 1;
    }

    /** Temporary access to the connectee for testing purposes. Real usage
        will be through the Connector (and Input) interfaces. 
        For example, Input should short circuit to its Output's getValue()
        once it is connected.
    Return a const reference to the object connected to this Connector */
    const T& getConnectee() const {
        return connectee.getRef();
    }

    /** Connect this Connector to the provided connectee object */
    void connect(const Object& object) override {
        const T* objT = dynamic_cast<const T*>(&object);
        if (objT) {
            connectee = *objT;

            std::string objPathName = objT->getFullPathName();
            std::string ownerPathName = getOwner().getFullPathName();

            // This can happen when top level components like a Joint and Body
            // have the same name like a pelvis Body and pelvis Joint that
            // connects that connects to a Body of the same name.
            if(objPathName == ownerPathName)
                set_connectee_name(objPathName);
            else { // otherwise store the relative path name to the object
                std::string relPathName = objT->getRelativePathName(getOwner());
                set_connectee_name(relPathName);
            }
        }
        else {
            std::stringstream msg;
            msg << "Connector::connect(): ERR- Cannot connect '" << object.getName()
                << "' of type " << object.getConcreteClassName() << ". Connector requires "
                << getConnecteeTypeName() << ".";
            throw Exception(msg.str(), __FILE__, __LINE__);
        }
    }

    /** Connect this Connector given its connectee_name property  */
    void findAndConnect(const Component& root) override;

    void disconnect() override {
        connectee.reset(nullptr);
    }
    
    /** Derived classes must satisfy this Interface */
    /** get the type of object this connector connects to*/
    std::string getConnecteeTypeName() const override
    { return T::getClassName(); }

    SimTK_DOWNCAST(Connector, AbstractConnector);

private:
    mutable SimTK::ReferencePtr<const T> connectee;
}; // END class Connector<T>


/** A specialized Connector that connects to an Output signal is an Input.
    An AbstractInput enables maintenance of a list of unconnected Inputs. 
    An Input can either be a single-value Input or a list Input. A list Input
    can connect to multiple (Output) Channels.
*/
class OSIMCOMMON_API AbstractInput : public AbstractConnector {
    OpenSim_DECLARE_ABSTRACT_OBJECT(AbstractInput, AbstractConnector);
public:
    /** Default constructor */
    AbstractInput() : AbstractConnector() {}
    /** Convenience constructor
    Create an AbstractInput (Connector) that connects only to an AbstractOutput
    specified by name and stage at which it should be connected.
    @param name             name of the dependent (Abstract)Output.
    @param connectAtStage   Stage at which Input should be connected.
    @param isList           Whether this Input can have multiple connectees.
    @param owner The component that contains this input. */
    AbstractInput(const std::string& name,
                  const SimTK::Stage& connectAtStage,
                  bool isList, const Component& owner) :
        AbstractConnector(name, connectAtStage, isList, owner) {}

    virtual ~AbstractInput() {}

    // Connector interface
    void connect(const Object& object) override {
        std::stringstream msg;
        msg << "Input::connect(): ERR- Cannot connect '" << object.getName()
            << "' of type " << object.getConcreteClassName() <<
            ". Input can only connect to an Output.";
        throw Exception(msg.str(), __FILE__, __LINE__);
    }

    /** Input-specific Connect. Connect this Input to a single-value Output or
     if this is a list Input and the output is a list Output, connect to 
     all the channels of the Output.
     You can optionally provide an annotation of the output that is specific
     to its use by the component that owns this input. If this method
     connects to multiple channels, the annotation will be used for all 
     the channels. */
    virtual void connect(const AbstractOutput& output,
                         const std::string& annotation = "") = 0;
    /** Connect to a single output channel. This can be used with either
    single-value or list Inputs.
    You can optionally provide an annotation of the output that is specific
    to its use by the component that owns this input. */
    virtual void connect(const AbstractChannel& channel,
                         const std::string& annotation = "") = 0;
    
    /** An Annotation is a description of a channel that is specific to how
        this input should use that channel. For example, the component
        containing this Input might expect the annotations to be the names
        of markers in the model.
        If no annotation was provided when connecting,
        the annotation is the name of the channel.
        If this is a list input, you must specify the specific Channel you want.*/
    virtual const std::string& getAnnotation(int index = -1) const = 0;
    // TODO what's the best way to serialize annotations?
    
//=============================================================================
};  // END class AbstractInput


/** An Input<Y> must be connected by an Output<Y> */
template<class T>
class  Input : public AbstractInput {
    OpenSim_DECLARE_CONCRETE_OBJECT(Input, AbstractInput);
public:

    typedef typename Output<T>::Channel Channel;

    typedef std::vector<SimTK::ReferencePtr<const Channel>> ChannelList;
    typedef std::vector<std::string> AnnotationList;
    
    /** Default constructor */
    Input() : AbstractInput() {}
    /** Convenience constructor
    Create an Input<T> (Connector) that can only connect to an Output<T>
    name and stage at which it should be connected.
    @param name             name of the Output dependency.
    @param connectAtStage   Stage at which Input should be connected.
    @param isList           Whether this Input can have multiple connectees.
    @param owner            The component that contains this input. */
    Input(const std::string& name, const SimTK::Stage& connectAtStage,
          bool isList, const Component& owner) :
        AbstractInput(name, connectAtStage, isList, owner) {}

    /** Connect this Input to the provided (Abstract)Output.
     */
    void connect(const AbstractOutput& output,
                 const std::string& annotation = "") override {
        const auto* outT = dynamic_cast<const Output<T>*>(&output);
        if (outT) {
            if (!isListConnector()) {
                if (outT->isListOutput()) {
                    throw Exception("Non-list input cannot connect to list output");
                }
                // Remove the existing connecteee (if it exists).
                disconnect();
            }
            // For a non-list connector, there will only be one channel.
            for (const auto& chan : outT->getChannels()) {
                _connectees.push_back(SimTK::ReferencePtr<const Channel>(&chan.second));
                // Use the same annotation for each channel.
                if (annotation.empty())
                    _annotations.push_back(chan.second.getChannelName());
                else _annotations.push_back(annotation);
            }
        }
        else {
            std::stringstream msg;
            msg << "Input::connect(): ERR- Cannot connect '" << output.getName()
            << "' of type Output<" << output.getTypeName() << ">. Input requires "
            << getConnecteeTypeName() << ".";
            throw Exception(msg.str(), __FILE__, __LINE__);
        }
    }
    
    void connect(const AbstractChannel& channel,
                 const std::string& annotation = "") override {
        const auto* chanT = dynamic_cast<const Channel*>(&channel);
        if (chanT) {
            if (!isListConnector()) {
                // Remove the existing connecteee (if it exists).
                disconnect();
            }
            _connectees.push_back(SimTK::ReferencePtr<const Channel>(chanT));
            if (annotation.empty()) _annotations.push_back(chanT->getChannelName());
            else _annotations.push_back(annotation);
        }
        else {
            std::stringstream msg;
            msg << "Input::connect(): ERR- Cannot connect '" << chanT.getFullPathName()
            << "' of type Output<" << output.getTypeName() << ">::Channel. Input requires "
            << getConnecteeTypeName() << ".";
            throw Exception(msg.str(), __FILE__, __LINE__);
        }
    }
    
    void disconnect() override {
        _connectees.clear();
        _annotations.clear();
    }
    
    bool isConnected() const override {
        return !_connectees.empty();
    }
    
    size_t getNumConnectees() const override {
        return _connectees.size();
    }
    
    std::string getConnecteeTypeName() const override
    { return SimTK::NiceTypeName<Output<T>>::namestr(); }

    /**Get the value of this Input when it is connected. Redirects to connected
       Output<T>'s getValue() with minimal overhead. If this is a list input,
       you must specify the specific Channel whose value you want. */
    const T& getValue(const SimTK::State &state, int index=-1) const {
        if (index < 0) {
            if (!isListConnector()) index = 0;
            else throw Exception("Input<T>::getValue(): an index must be "
                                 "provided for a list input.");
        }
        // TODO remove this check in order to improve speed?
        OPENSIM_THROW_IF(index >= getNumConnectees(),
                         IndexOutOfRange<int>,
                         index, 0, (int)getNumConnectees() - 1);
        return _connectees[index].getRef().getValue(state);
    }
    
    /** If this is a list input, you must specify the specific Channel you want.*/
    const Channel& getChannel(int index=-1) const {
        if (index == -1) {
            if (!isListConnector()) index = 0;
            else throw Exception("Input<T>::getChannel(): an index must be "
                                 "provided for a list input.");
        }
        OPENSIM_THROW_IF(index >= getNumConnectees(),
                         IndexOutOfRange<int>,
                         index, 0, (int)getNumConnectees() - 1);
        return _connectees[index].getRef();
    }
    
    const std::string& getAnnotation(int index = -1) const override {
        if (index == -1) {
            if (!isListConnector()) index = 0;
            else throw Exception("Input<T>::getAnnotation(): an index must be "
                                 "provided for a list input.");
        }
        OPENSIM_THROW_IF(index >= getNumConnectees(),
                         IndexOutOfRange<int>,
                         index, 0, (int)getNumConnectees() - 1);
        return _annotations[index];
    }
    
    /** Access the values of all the channels connected to this Input as a 
    SimTK::Vector_<T>. The elements are in the same order as the channels.
    */
    SimTK::Vector_<T> getVector(const SimTK::State& state) const {
        SimTK::Vector_<T> v(getNumConnectees());
        for (int ichan = 0; ichan < _connectees.size(); ++ichan) {
            v[ichan] = _connectees[ichan]->getValue(state);
        }
        return v;
    }
    
    /** Get const access to the channels connected to this input.
        You can use this to iterate through the channels.
        @code{.cpp}
        for (const auto& chan : getChannels()) {
            std::cout << chan.getValue(state) << std::endl;
        }
        @endcode
    */
    const ChannelList& getChannels() const {
        return _connectees;
    }
    
    SimTK_DOWNCAST(Input, AbstractInput);

private:
    SimTK::ResetOnCopy<ChannelList> _connectees;
    SimTK::ResetOnCopy<AnnotationList> _annotations;
}; // END class Input<Y>

/// @name Creating Inputs for your Component
/// Use these macros at the top of your component class declaration,
/// near where you declare @ref Property properties.
/// @{
/** Create a socket for this component's dependence on an output signal from
 *  another component. It is a placeholder for an Output that can be connected
 *  to it. An output must have the same type T as an input to be connected
 *  to it. You must also specify the stage at which you require this input
 *  quantity. The comment should describe how the input quantity is used.
 *
 *  An Input declared with this macro can connect to only one Output.
 *
 *  Here's an example for using this macro:
 *  @code{.cpp}
 *  class MyComponent : public Component {
 *  public:
 *      OpenSim_DECLARE_INPUT(emg, double, SimTK::Stage::Velocity, "For validation.");
 *      ...
 *  };
 *  @endcode
 * @see Component::constructInput()
 * @relates OpenSim::Input
 */
#define OpenSim_DECLARE_INPUT(iname, T, istage, comment)                    \
    /** @name Inputs                                                     */ \
    /** @{                                                               */ \
    /** comment                                                          */ \
    /** This input is needed at stage istage.                            */ \
    /** This input was generated with the                                */ \
    /** #OpenSim_DECLARE_INPUT macro.                                    */ \
    OpenSim_DOXYGEN_Q_PROPERTY(T, iname)                                    \
    /** @}                                                               */ \
    /** @cond                                                            */ \
    bool _has_input_##iname { constructInput<T>(#iname, istage) };          \
    /** @endcond                                                         */
    
/** Create a list input, which can connect to more than one Channel. This
 * makes sense for components like reporters that can handle a flexible
 * number of input values. 
 *
 * @see Component::constructInput()
 * @relates OpenSim::Input
 */
#define OpenSim_DECLARE_LIST_INPUT(iname, T, istage, comment)               \
    /** @name Inputs (list)                                              */ \
    /** @{                                                               */ \
    /** comment                                                          */ \
    /** This input can connect to multiple outputs, all of which are     */ \
    /** needed at stage istage.                                          */ \
    /** This input was generated with the                                */ \
    /** #OpenSim_DECLARE_LIST_INPUT macro.                               */ \
    OpenSim_DOXYGEN_Q_PROPERTY(T, iname)                                    \
    /** @}                                                               */ \
    /** @cond                                                            */ \
    bool _has_input_##iname { constructInput<T>(#iname, istage, true) };    \
    /** @endcond                                                         */
/// @}

} // end of namespace OpenSim

#endif  // OPENSIM_COMPONENT_CONNECTOR_H_
